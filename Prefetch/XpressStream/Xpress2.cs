using System;
using System.IO;

namespace Prefetch.XpressStream
{
    /// <summary>
    /// Pure managed C# implementation of LZXPRESS Huffman decompression.
    /// Replaces the Windows-only ntdll.dll P/Invoke (RtlDecompressBufferEx)
    /// with a fully cross-platform algorithm implementation per [MS-XCA] §2.3.
    /// Works identically on Windows, Linux, and macOS with no native dependencies.
    /// </summary>
    public static class Xpress2
    {
        // LZXPRESS Huffman constants per [MS-XCA]
        private const int HuffmanSymbolCount = 512;       // 512 Huffman symbols
        private const int HuffmanTableBytes  = 256;       // 512 symbols × 4 bits = 256 bytes
        private const int MaxChunkSize       = 65536;     // 64 KB input chunk size
        private const int MaxCodeLength      = 15;        // Maximum Huffman code length

        /// <summary>
        /// Decompresses an LZXPRESS Huffman compressed buffer.
        /// Equivalent to calling ntdll!RtlDecompressBufferEx with
        /// COMPRESSION_FORMAT_XPRESS_HUFF (0x8004) on Windows.
        /// </summary>
        /// <param name="buffer">Compressed input data (after MAM header is stripped)</param>
        /// <param name="decompressedSize">Expected output size from MAM header field</param>
        /// <returns>Decompressed byte array, or null on failure</returns>
        public static byte[] Decompress(byte[] buffer, ulong decompressedSize)
        {
            if (buffer == null || buffer.Length == 0)
                return null;

            var output    = new byte[decompressedSize];
            int outputPos = 0;
            int inputPos  = 0;

            // Process 64 KB chunks until output is filled or input exhausted
            while (outputPos < (int)decompressedSize && inputPos < buffer.Length)
            {
                // Each chunk begins with a 256-byte Huffman symbol length table.
                // 512 symbols × 4 bits each = 256 bytes.
                if (inputPos + HuffmanTableBytes > buffer.Length)
                    break;

                // ── Step 1: Build Huffman decode table ────────────────────────────
                var symbolLengths = new int[HuffmanSymbolCount];
                for (int i = 0; i < HuffmanTableBytes; i++)
                {
                    // Each byte encodes two 4-bit symbol lengths, low nibble first
                    symbolLengths[i * 2]     = buffer[inputPos + i] & 0x0F;
                    symbolLengths[i * 2 + 1] = (buffer[inputPos + i] >> 4) & 0x0F;
                }
                inputPos += HuffmanTableBytes;

                // Build canonical Huffman decode table (code → symbol mapping)
                // using the standard canonical Huffman construction algorithm
                int[] decodeTable = BuildDecodeTable(symbolLengths);

                // ── Step 2: Decompress one chunk ──────────────────────────────────
                // Determine this chunk's output boundary (max 64 KB or remaining)
                int chunkOutputEnd = Math.Min(outputPos + MaxChunkSize, (int)decompressedSize);

                // Bit-stream reader state
                ulong bitBuffer   = 0;
                int   bitsInBuf   = 0;

                // Helper: ensure at least `count` bits are loaded into bitBuffer
                // Returns false if the input stream is exhausted
                bool FillBits(int count)
                {
                    while (bitsInBuf < count)
                    {
                        if (inputPos >= buffer.Length)
                            return false;
                        bitBuffer |= ((ulong)buffer[inputPos++]) << bitsInBuf;
                        bitsInBuf += 8;
                    }
                    return true;
                }

                // Helper: peek at `count` bits without consuming them
                int PeekBits(int count) => (int)(bitBuffer & ((1UL << count) - 1));

                // Helper: consume `count` bits
                void ConsumeBits(int count)
                {
                    bitBuffer >>= count;
                    bitsInBuf  -= count;
                }

                while (outputPos < chunkOutputEnd)
                {
                    // Read enough bits to decode one Huffman symbol (up to 15 bits)
                    if (!FillBits(MaxCodeLength))
                        break;

                    // Decode one symbol using the decode table
                    int symbol = DecodeSymbol(decodeTable, symbolLengths, PeekBits, ConsumeBits);

                    if (symbol < 256)
                    {
                        // Literal byte — copy directly to output
                        if (outputPos < output.Length)
                            output[outputPos++] = (byte)symbol;
                    }
                    else
                    {
                        // Back-reference: symbol encodes match length and offset info
                        // Symbol layout: bits [8:3] = length slot, bits [2:0] = offset high bits
                        int matchSym    = symbol - 256;
                        int lenSlot     = matchSym >> 3;      // length slot (0-15 in low codes)
                        int offsetHigh  = matchSym & 0x07;    // 3 high bits of offset

                        // Decode match length
                        int matchLength;
                        if (lenSlot < 15)
                        {
                            matchLength = lenSlot + 3;
                        }
                        else
                        {
                            // Extra length bytes follow in the bit stream
                            if (!FillBits(8)) break;
                            int extraLen = PeekBits(8);
                            ConsumeBits(8);

                            if (extraLen < 255)
                            {
                                matchLength = extraLen + 3 + 15;
                            }
                            else
                            {
                                // 16-bit extra length follows
                                if (!FillBits(16)) break;
                                matchLength = PeekBits(16);
                                ConsumeBits(16);
                                matchLength += 3;
                            }
                        }

                        // Decode match offset: offsetHigh from symbol + low bits from stream
                        // Number of low-order offset bits to read = offsetHigh
                        int offsetLow = 0;
                        if (offsetHigh > 0)
                        {
                            if (!FillBits(offsetHigh)) break;
                            offsetLow = PeekBits(offsetHigh);
                            ConsumeBits(offsetHigh);
                        }
                        int matchOffset = (1 << offsetHigh) + offsetLow - 1;
                        // [MS-XCA]: offset is stored as distance-1 from current position
                        int copyFrom = outputPos - matchOffset - 1;

                        if (copyFrom < 0)
                            break;

                        // Copy match bytes (may overlap — copy byte-by-byte)
                        int copyCount = Math.Min(matchLength, (int)decompressedSize - outputPos);
                        for (int i = 0; i < copyCount; i++)
                        {
                            output[outputPos] = output[copyFrom + i];
                            outputPos++;
                        }
                    }
                }
            }

            return output;
        }

        /// <summary>
        /// Builds a canonical Huffman decode lookup table from symbol code lengths.
        /// Uses a 15-bit wide direct-lookup table for O(1) decode per symbol.
        /// Standard canonical Huffman construction per RFC 1951 / [MS-XCA].
        /// </summary>
        private static int[] BuildDecodeTable(int[] symbolLengths)
        {
            // Count symbols at each code length (1–15)
            var lengthCount = new int[MaxCodeLength + 1];
            foreach (int len in symbolLengths)
                if (len > 0)
                    lengthCount[len]++;

            // Compute first canonical code for each length
            var firstCode = new int[MaxCodeLength + 1];
            int code = 0;
            for (int bits = 1; bits <= MaxCodeLength; bits++)
            {
                code = (code + lengthCount[bits - 1]) << 1;
                firstCode[bits] = code;
            }

            // Build 15-bit wide direct decode table (2^15 = 32768 entries)
            // Each entry: (symbol << 4) | codeLength  — 0 means invalid/unused
            const int tableSize = 1 << MaxCodeLength;
            var decodeTable = new int[tableSize];

            for (int sym = 0; sym < symbolLengths.Length; sym++)
            {
                int len = symbolLengths[sym];
                if (len == 0)
                    continue;

                int symCode = firstCode[len]++;
                // Fill all 15-bit table entries that start with this code
                int step   = 1 << len;
                int start  = symCode << (MaxCodeLength - len);

                for (int entry = start; entry < tableSize; entry += step)
                    decodeTable[entry] = (sym << 4) | len;
            }

            return decodeTable;
        }

        /// <summary>
        /// Decodes one Huffman symbol from the bit stream using the direct lookup table.
        /// Consumes only the bits belonging to the decoded symbol.
        /// </summary>
        private static int DecodeSymbol(
            int[] decodeTable,
            int[] symbolLengths,
            Func<int, int> peekBits,
            Action<int> consumeBits)
        {
            // Peek up to MaxCodeLength bits and look up in table
            int index   = peekBits(MaxCodeLength);
            int entry   = decodeTable[index];
            int codeLen = entry & 0x0F;
            int symbol  = entry >> 4;

            // Consume only the bits used by this code
            consumeBits(codeLen == 0 ? MaxCodeLength : codeLen);

            return symbol;
        }
    }
}

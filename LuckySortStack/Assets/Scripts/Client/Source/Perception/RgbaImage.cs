using System;
using System.IO;
using System.IO.Compression;
using System.Numerics;

namespace LuckySortStack.Perception
{
	// 8-bit RGBA image, row 0 at the TOP (the PinholeCamera pixel convention). Captures whose rows arrive bottom-up are
	// flipped on the way in (FromCapture). Also writes PNGs and draws simple debug annotations, so the vision spike and
	// the tests can save what the camera saw without any engine or imaging library.
	public sealed class RgbaImage
	{
		public readonly int Width;
		public readonly int Height;
		public readonly byte[] Data;   // Width * Height * 4, row-major, RGBA

		public RgbaImage(int width, int height, byte[]? data = null)
		{
			if (width <= 0 || height <= 0) throw new ArgumentOutOfRangeException(nameof(width));
			Width = width;
			Height = height;
			Data = data ?? new byte[width * height * 4];
			if (Data.Length < width * height * 4) throw new ArgumentException("pixel buffer too small", nameof(data));
		}

		// Wrap a capture buffer. bottomUp = the buffer's first row is the bottom of the picture (OpenGL readback order).
		public static RgbaImage FromCapture(byte[] rgba, int width, int height, bool bottomUp)
		{
			var img = new RgbaImage(width, height);
			int stride = width * 4;
			for (int y = 0; y < height; y++)
				Buffer.BlockCopy(rgba, (bottomUp ? height - 1 - y : y) * stride, img.Data, y * stride, stride);
			return img;
		}

		public RgbaImage Clone() => new RgbaImage(Width, Height, (byte[])Data.Clone());

		public bool Contains(int x, int y) => x >= 0 && y >= 0 && x < Width && y < Height;

		public (byte R, byte G, byte B) Get(int x, int y)
		{
			int i = (y * Width + x) * 4;
			return (Data[i], Data[i + 1], Data[i + 2]);
		}

		public void Set(int x, int y, byte r, byte g, byte b, byte a = 255)
		{
			if (!Contains(x, y)) return;
			int i = (y * Width + x) * 4;
			Data[i] = r; Data[i + 1] = g; Data[i + 2] = b; Data[i + 3] = a;
		}

		public void Set(int x, int y, (byte R, byte G, byte B) c) => Set(x, y, c.R, c.G, c.B);

		// ── Debug drawing ───────────────────────────────────────────────────

		public void DrawLine(Vector2 a, Vector2 b, (byte R, byte G, byte B) c)
		{
			if (float.IsNaN(a.X) || float.IsNaN(b.X)) return;
			int steps = (int)MathF.Ceiling(MathF.Max(MathF.Abs(b.X - a.X), MathF.Abs(b.Y - a.Y))) + 1;
			if (steps > 20000) return;
			for (int s = 0; s <= steps; s++)
			{
				Vector2 p = Vector2.Lerp(a, b, s / (float)steps);
				Set((int)MathF.Floor(p.X), (int)MathF.Floor(p.Y), c);
			}
		}

		public void DrawCross(Vector2 p, int half, (byte R, byte G, byte B) c)
		{
			DrawLine(p - new Vector2(half, 0), p + new Vector2(half, 0), c);
			DrawLine(p - new Vector2(0, half), p + new Vector2(0, half), c);
		}

		public void DrawPolygon(Vector2[] points, (byte R, byte G, byte B) c)
		{
			for (int i = 0; i < points.Length; i++) DrawLine(points[i], points[(i + 1) % points.Length], c);
		}

		// ── PNG ─────────────────────────────────────────────────────────────

		public void SavePng(string path)
		{
			string? dir = Path.GetDirectoryName(path);
			if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
			using var file = File.Create(path);
			WritePng(file);
		}

		public byte[] ToPng()
		{
			using var ms = new MemoryStream();
			WritePng(ms);
			return ms.ToArray();
		}

		// Minimal PNG: 8-bit RGBA, no filtering, one zlib-compressed IDAT.
		public void WritePng(Stream output)
		{
			output.Write(new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 }, 0, 8);

			var ihdr = new byte[13];
			WriteBE(ihdr, 0, (uint)Width);
			WriteBE(ihdr, 4, (uint)Height);
			ihdr[8] = 8;    // bit depth
			ihdr[9] = 6;    // colour type RGBA
			WriteChunk(output, "IHDR", ihdr);

			using (var raw = new MemoryStream())
			{
				using (var z = new ZLibStream(raw, CompressionLevel.Fastest, leaveOpen: true))
				{
					int stride = Width * 4;
					for (int y = 0; y < Height; y++)
					{
						z.WriteByte(0);   // filter: none
						z.Write(Data, y * stride, stride);
					}
				}
				WriteChunk(output, "IDAT", raw.ToArray());
			}
			WriteChunk(output, "IEND", Array.Empty<byte>());
		}

		private static void WriteChunk(Stream s, string type, byte[] data)
		{
			var len = new byte[4];
			WriteBE(len, 0, (uint)data.Length);
			s.Write(len, 0, 4);
			var typeBytes = System.Text.Encoding.ASCII.GetBytes(type);
			s.Write(typeBytes, 0, 4);
			s.Write(data, 0, data.Length);
			uint crc = Crc32(typeBytes, 0xFFFFFFFFu);
			crc = Crc32(data, crc) ^ 0xFFFFFFFFu;
			var crcBytes = new byte[4];
			WriteBE(crcBytes, 0, crc);
			s.Write(crcBytes, 0, 4);
		}

		private static void WriteBE(byte[] b, int o, uint v)
		{
			b[o] = (byte)(v >> 24); b[o + 1] = (byte)(v >> 16); b[o + 2] = (byte)(v >> 8); b[o + 3] = (byte)v;
		}

		private static uint[]? s_CrcTable;

		private static uint Crc32(byte[] data, uint crc)
		{
			if (s_CrcTable == null)
			{
				var t = new uint[256];
				for (uint n = 0; n < 256; n++)
				{
					uint c = n;
					for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
					t[n] = c;
				}
				s_CrcTable = t;
			}
			foreach (byte b in data) crc = s_CrcTable[(crc ^ b) & 0xFF] ^ (crc >> 8);
			return crc;
		}
	}
}

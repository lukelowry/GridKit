#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace GridKit
{
  namespace Utilities
  {
    namespace Detail
    {
      inline constexpr std::array<std::uint32_t, 64> sha256_k{
          0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
          0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
          0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
          0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
          0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
          0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
          0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
          0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
          0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
          0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
          0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
          0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
          0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
          0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
          0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
          0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

      inline constexpr std::uint32_t rotr(std::uint32_t value, std::uint32_t bits)
      {
        return (value >> bits) | (value << (32u - bits));
      }

      inline std::uint32_t readBigEndian32(const std::uint8_t* data)
      {
        return (static_cast<std::uint32_t>(data[0]) << 24u)
               | (static_cast<std::uint32_t>(data[1]) << 16u)
               | (static_cast<std::uint32_t>(data[2]) << 8u)
               | static_cast<std::uint32_t>(data[3]);
      }

      inline void appendBigEndian64(std::vector<std::uint8_t>& data, std::uint64_t value)
      {
        for (int shift = 56; shift >= 0; shift -= 8)
        {
          data.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffu));
        }
      }

      inline std::string hexDigest(const std::array<std::uint32_t, 8>& words)
      {
        static constexpr char digits[] = "0123456789abcdef";
        std::string           result;
        result.reserve(64);
        for (const auto word : words)
        {
          for (int shift = 28; shift >= 0; shift -= 4)
          {
            result.push_back(digits[(word >> shift) & 0x0fu]);
          }
        }
        return result;
      }
    } // namespace Detail

    inline std::string sha256Hex(std::span<const std::uint8_t> input)
    {
      std::vector<std::uint8_t> data(input.begin(), input.end());
      const std::uint64_t       bit_length = static_cast<std::uint64_t>(data.size()) * 8u;

      data.push_back(0x80u);
      while ((data.size() % 64u) != 56u)
      {
        data.push_back(0u);
      }
      Detail::appendBigEndian64(data, bit_length);

      std::array<std::uint32_t, 8> h{
          0x6a09e667u,
          0xbb67ae85u,
          0x3c6ef372u,
          0xa54ff53au,
          0x510e527fu,
          0x9b05688cu,
          0x1f83d9abu,
          0x5be0cd19u};

      for (std::size_t chunk = 0; chunk < data.size(); chunk += 64u)
      {
        std::array<std::uint32_t, 64> w{};
        for (std::size_t i = 0; i < 16u; ++i)
        {
          w[i] = Detail::readBigEndian32(data.data() + chunk + 4u * i);
        }
        for (std::size_t i = 16u; i < 64u; ++i)
        {
          const auto s0 = Detail::rotr(w[i - 15u], 7u) ^ Detail::rotr(w[i - 15u], 18u)
                          ^ (w[i - 15u] >> 3u);
          const auto s1 = Detail::rotr(w[i - 2u], 17u) ^ Detail::rotr(w[i - 2u], 19u)
                          ^ (w[i - 2u] >> 10u);
          w[i] = w[i - 16u] + s0 + w[i - 7u] + s1;
        }

        auto a = h[0];
        auto b = h[1];
        auto c = h[2];
        auto d = h[3];
        auto e = h[4];
        auto f = h[5];
        auto g = h[6];
        auto i = h[7];

        for (std::size_t round = 0; round < 64u; ++round)
        {
          const auto s1    = Detail::rotr(e, 6u) ^ Detail::rotr(e, 11u) ^ Detail::rotr(e, 25u);
          const auto ch    = (e & f) ^ ((~e) & g);
          const auto temp1 = i + s1 + ch + Detail::sha256_k[round] + w[round];
          const auto s0    = Detail::rotr(a, 2u) ^ Detail::rotr(a, 13u) ^ Detail::rotr(a, 22u);
          const auto maj   = (a & b) ^ (a & c) ^ (b & c);
          const auto temp2 = s0 + maj;

          i = g;
          g = f;
          f = e;
          e = d + temp1;
          d = c;
          c = b;
          b = a;
          a = temp1 + temp2;
        }

        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += i;
      }

      return Detail::hexDigest(h);
    }
  } // namespace Utilities
} // namespace GridKit

Toolchain built from gcc-arm-none-eabi-4_9-2015q3-20150921-src.tar.bz2 at
https://launchpad.net/gcc-arm-embedded.  To build follow the instructions
in How-to-build-toolchain.pdf from that tarball.  After the sub-tarballs are
unpacked ("find -name '*.tar.*' | xargs -I% tar -xf %"), apply
gcc_big_endian.patch to src/gcc.

#!/bin/sh
# Build a native AVR toolchain (binutils + gcc + avr-libc) from official
# sources. Useful on hosts where Arduino only ships x86_64 compiler binaries
# (e.g. Apple Silicon without Rosetta). nanoemu itself does NOT need this.
#
#   tools/build-avr-toolchain.sh              # installs into ~/.local/avr
#   PREFIX=/opt/avr JOBS=4 tools/build-avr-toolchain.sh
#
# Requirements: a C/C++ compiler, make, curl, and gmp/mpfr/libmpc
# (installed via Homebrew on macOS, downloaded by GCC's script elsewhere).
set -e

BINUTILS_V=2.43
GCC_V=14.2.0
LIBC_V=2.2.1

PREFIX=${PREFIX:-$HOME/.local/avr}
BUILD=${BUILD:-${TMPDIR:-/tmp}/avr-toolchain-build}
JOBS=${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)}

say() { printf '\n==> %s\n' "$*"; }
fetch() { [ -s "$2" ] || { curl -fL --retry 3 -o "$2.part" "$1" && mv "$2.part" "$2"; }; }

mkdir -p "$BUILD" "$PREFIX"
cd "$BUILD"

say "downloading sources into $BUILD"
fetch "https://ftp.gnu.org/gnu/binutils/binutils-$BINUTILS_V.tar.xz" "binutils-$BINUTILS_V.tar.xz"
fetch "https://ftp.gnu.org/gnu/gcc/gcc-$GCC_V/gcc-$GCC_V.tar.xz" "gcc-$GCC_V.tar.xz"
fetch "https://github.com/avrdudes/avr-libc/releases/download/avr-libc-$(echo $LIBC_V | tr . _)-release/avr-libc-$LIBC_V.tar.bz2" "avr-libc-$LIBC_V.tar.bz2"

for f in binutils-$BINUTILS_V.tar.xz gcc-$GCC_V.tar.xz avr-libc-$LIBC_V.tar.bz2; do
    d=${f%.tar.*}
    [ -d "$d" ] || { say "extracting $f"; tar xf "$f"; }
done

GMP_OPTS=""
if [ "$(uname)" = Darwin ] && command -v brew >/dev/null; then
    say "installing gmp mpfr libmpc (Homebrew)"
    brew list gmp >/dev/null 2>&1 && brew list mpfr >/dev/null 2>&1 && brew list libmpc >/dev/null 2>&1 \
        || brew install gmp mpfr libmpc
    GMP_OPTS="--with-gmp=$(brew --prefix gmp) --with-mpfr=$(brew --prefix mpfr) --with-mpc=$(brew --prefix libmpc)"
else
    (cd gcc-$GCC_V && [ -d gmp ] || ./contrib/download_prerequisites)
fi

export PATH="$PREFIX/bin:$PATH"

say "binutils $BINUTILS_V"
rm -rf build-binutils && mkdir build-binutils && cd build-binutils
../binutils-$BINUTILS_V/configure --prefix="$PREFIX" --target=avr \
    --disable-nls --disable-werror --with-system-zlib --disable-gdb --disable-gdbserver --disable-sim \
    --disable-gprofng --disable-libdecnumber --disable-readline
make -j"$JOBS" MAKEINFO=true
make install MAKEINFO=true
cd ..

say "gcc $GCC_V (C, C++)"
rm -rf build-gcc && mkdir build-gcc && cd build-gcc
../gcc-$GCC_V/configure --prefix="$PREFIX" --target=avr \
    --enable-languages=c,c++ --disable-nls --with-system-zlib --disable-libssp --disable-libcc1 \
    --disable-shared --disable-threads --with-dwarf2 --with-avrlibc $GMP_OPTS
make -j"$JOBS" MAKEINFO=true all-gcc all-target-libgcc
make MAKEINFO=true install-gcc install-target-libgcc
cd ..

say "avr-libc $LIBC_V"
cd avr-libc-$LIBC_V
./configure --prefix="$PREFIX" --build="$(./config.guess)" --host=avr
make -j"$JOBS" MAKEINFO=true
make install MAKEINFO=true
cd ..

say "done: $PREFIX/bin/avr-gcc"
"$PREFIX/bin/avr-gcc" --version | head -1

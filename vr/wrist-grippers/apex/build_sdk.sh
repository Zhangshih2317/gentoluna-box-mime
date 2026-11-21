#!/bin/bash
set -euo pipefail
# Usage: build_sdk.sh pristine-C_SDK build-dir [isolated-GCC-prefix]
src=$(realpath "$1")
mkdir -p "$2"
out=$(realpath "$2")
here=$(cd -- "$(dirname "$0")" && pwd)
export LC_ALL=C
[[ ! -e "$out/src" ]] || { echo "Use a fresh build directory" >&2; exit 2; }
cp -a "$src" "$out/src"
cd "$out/src"
dirs=(Common Kinematics Kinematics/{ArmKinematics,BaseMath,DynaIdent,KineCommon,MotionPlanner,SkyeBodyKinematics} FileClient L0Control L1Robot)
sources=(./L0Control/*.cpp ./FileClient/*.cpp ./L1Robot/*.cpp ./Kinematics/*.cpp
    ./Kinematics/{ArmKinematics,BaseMath,DynaIdent,KineCommon,MotionPlanner,SkyeBodyKinematics}/*.cpp)
flags=(-Wall -O2 -fPIC -DCMPL_LIN)
includes=()
for dir in "${dirs[@]}"; do flags+=("-I./$dir"); includes+=("$PWD/$dir"); done
compiler=(g++)
if [[ $# == 3 ]]; then
    prefix=$(realpath "$3")
    compiler=("$prefix/usr/bin/aarch64-linux-gnu-g++-11"
        "-B$prefix/usr/lib/gcc/aarch64-linux-gnu/11/" "-B/usr/lib/gcc/aarch64-linux-gnu/11/"
        -isystem /usr/include/c++/11 -isystem /usr/include/aarch64-linux-gnu/c++/11
        -isystem /usr/include/c++/11/backward -isystem /usr/lib/gcc/aarch64-linux-gnu/11/include)
fi
build_obj() {
    "${compiler[@]}" "${flags[@]}" -c "$1" -o "$out/${1##*/}.o"
}
objects=()
for file in "${sources[@]}"; do
    objects+=("$out/${file##*/}.o")
    build_obj "$file"
done
"${compiler[@]}" -shared "${objects[@]}" -lpthread -lrt -o "$out/baseline.so"
stock=/usr/local/lib/libGentoSDK.so
for section in .text .rodata .data; do
    objcopy --dump-section "$section=$out/stock$section" "$stock" /dev/null
    objcopy --dump-section "$section=$out/base$section" "$out/baseline.so" /dev/null
    cmp "$out/stock$section" "$out/base$section"
done
diff -u <(nm -DS --defined-only "$stock" | awk '{print $4,$2}' | sort) \
        <(nm -DS --defined-only "$out/baseline.so" | awk '{print $4,$2}' | sort)
echo "Matched stock SDK code, constant data, initialized data and exported symbol sizes"
sed -i 's/\r//g' L1Robot/L1Robot.cpp
git apply --recount --check "$here/sdk-wrist.patch"
git apply --recount "$here/sdk-wrist.patch"
build_obj ./L1Robot/L1Robot.cpp
cmake -S "$here" -B "$out/cmake" \
    "-DSDK_OBJECTS=$(IFS=';'; echo "${objects[*]}")" \
    "-DGENTO_INCLUDES=$(IFS=';'; echo "${includes[*]}")" -DCMAKE_BUILD_TYPE=Release
cmake --build "$out/cmake" -j2
comm -23 <(nm -D --defined-only "$stock" | awk '{print $3}' | sort -u) \
         <(nm -D --defined-only "$out/cmake/libGentoSDK.so" | awk '{print $3}' | sort -u) >"$out/missing-symbols"
[[ ! -s "$out/missing-symbols" ]]

g++ --version

#! https://stackoverflow.com/a/246128
SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
ROOT_DIR=$(realpath $SCRIPT_DIR/..)

echo "Got root of directory: $ROOT_DIR"

params="-Wall -I src/include -std=c++20"
if [ "$1" == "debug" ]
then
    params="$params -g -D DEBUG"
    echo "Building in debug mode"

elif [ "$1" == "profile-generate" ]
then
    params="$params -O3 -fprofile-generate"
    echo "Building in profile mode"

elif [ "$1" == "profile-use" ]
then
    params="$params -O3 -fprofile-use"
    echo "Building using generated profile data"

else
    params="$params -O3"
    echo "Building normally"
fi

mkdir -p build
echo "Compiling $ROOT_DIR/src/main.cpp to $ROOT_DIR/build/main.exe"
echo "Running \"g++ $params $ROOT_DIR/src/main.cpp -o $ROOT_DIR/build/main.exe\""
g++ $params $ROOT_DIR/src/main.cpp -o $ROOT_DIR/build/main.exe

g++ --version

#! https://stackoverflow.com/a/246128
SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
ROOT_DIR=$(realpath $SCRIPT_DIR/..)

echo "Got root of directory: $ROOT_DIR"
mkdir -p ROOT_DIR/build

if [ "$1" == "alglib" ]
then
    echo "Building ALGLIB"

    #! https://stackoverflow.com/a/9612560
    find $ROOT_DIR/extern/alglib-cpp/src/*.cpp -print0 | while read -d $'\0' file
    do
        command="g++ -c -O3 -D AE_CPU=AE_INTEL -m avx2 -m fma -std=c++20 $file -o $ROOT_DIR/build/$(basename ${file%.cpp}).o"
        echo "Running \"$command\""
        $command
    done

else
    echo "Building main.exe"
    params="-Wall -I src/include -I extern/alglib-cpp/src -std=c++20"
    if [ "$1" == "debug" ]
    then
        params="$params -g -D DEBUG"
        echo "Building in debug mode"

    else
        params="$params -O3"
        if [ "$1" == "profile-generate" ]
        then
            params="$params -f profile-generate"
            echo "Building in profile mode"

        elif [ "$1" == "profile-use" ]
        then
            params="$params -f profile-use"
            echo "Building using generated profile data"

        else
            echo "Building normally"

        fi

    fi

    command="g++ $params $ROOT_DIR/src/main.cpp $ROOT_DIR/build/*.o -o $ROOT_DIR/build/main.exe"
    echo "Running \"$command\""
    $command

fi

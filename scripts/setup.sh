SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
ROOT_DIR=$(realpath $SCRIPT_DIR/..)

mkdir $ROOT_DIR/result

mkdir $ROOT_DIR/.vscode
cp $ROOT_DIR/c_cpp_properties.json $ROOT_DIR/.vscode/c_cpp_properties.json

python3 -m venv $ROOT_DIR/.venv
$ROOT_DIR/.venv/bin/pip install -r $ROOT_DIR/requirements.txt

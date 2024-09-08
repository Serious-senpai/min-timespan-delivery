# min-timespan-delivery

[![Build and debug](https://github.com/Serious-senpai/soict-2024/actions/workflows/tests.yml/badge.svg)](https://github.com/Serious-senpai/soict-2024/actions/workflows/tests.yml)
[![Run algorithm](https://github.com/Serious-senpai/soict-2024/actions/workflows/solve.yml/badge.svg)](https://github.com/Serious-senpai/soict-2024/actions/workflows/solve.yml)
[![Lint](https://github.com/Serious-senpai/soict-2024/actions/workflows/lint.yml/badge.svg)](https://github.com/Serious-senpai/soict-2024/actions/workflows/lint.yml)

### Compile [ALGLIB library](https://www.alglib.net)
```bash
$ scripts/build.sh alglib
```

### Compile in debug mode
##### Compile C++ source files
```bash
$ scripts/build.sh debug
```

##### Run in debug mode
```bash
$ python scripts/in.py 6.5.1 -v | gdb --command=scripts/gdb.txt --return-child-result build/main.exe
```

### Compile in release mode
##### Compile C++ source files
```bash
$ scripts/build.sh
```

##### Prepare directory to store results
```bash
$ mkdir result
```

##### Run algorithm
```bash
$ python scripts/in.py 6.5.1 -v | build/main.exe | python scripts/out.py 6.5.1
```

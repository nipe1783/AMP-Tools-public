# AMP-Tools
### Professor: Morteza Lahijanian
### Developed by: Peter Amorese
Auto-grader, visualization, and other tools for ASEN 5254 (Algorithmic Motion Planning) at CU Boulder.

## Navigation

### Scripts
Two build scripts and a grader script are included for your convenience. You can call them using `bash`:
```
bash <script_name>.sh
```
 - `build.sh`: Compile your workspace. Flags: `-r` or `--rebuild` rebuilds from scratch.
 - `build_and_run.sh`: Compile your workspace and run your `main.cpp` executable (this script is auto-generated by the `build.sh` script, so make sure to run that one first). Flags: `-r` or `--rebuild` rebuilds from scratch.
 - `grade.sh`: Grade your homework. Flags: `--hw#` (e.g. `--hw2`) dictates which homework should be graded. Outputs a zip file `file_dump/out/hw#_report_card.zip` for you to submit. Make sure to call `HW#::grade()` somewhere in your main function. **SEE SUBMISSION INSTRUCTIONS BELOW**

After installation, verify the `build_and_run.sh` script works.

**NOTE:** The compiling your workspace will automatically compile the visualizer using your current Python interpreter. Make sure that you installed `requirements.txt` using the same interpreter that your build your project with. Otherwise you may experience missing modules. To check which interpreter you are using: `which python3`. If you switch your interpreter or virtual environment, e.g. `conda deactivate`/`conda activate`, you will need to rebuild from scratch (see flags above).

### Workspace `ws/`
The workspace direction `ws/` is where you will put all of your code. You will find a `main.cpp` file which contains your `main()` function. Feel free to add any headers `.h` and class definition files `.cpp` into that directory. They will automatically be linked to `main.cpp`.

### File IO `file_dump/`
If you are NOT using C++, this is directory contains an `in` and `out` folder for input and output files respectively.

### Include `include/`
This directory includes all of the header files you need. The files contain documentation that will help you understand the classes and methods. Do not edit these files, this will cause build errors.

### Build files `build/`
Here you will find the build artifacts (compiled binaries, etc.). You will generally not need to worry about files in here. If you would like to manually run your `main` executable, you will find it in `build/bin`. 

### Prebuilt executable tools `bin/`
After you build your project for the first time, this directory will automatically be generated. If you are NOT using C++, you can call these executables the generate workspaces, check solutions, etc. Any file input and output for these executables is done through the `file_dump/` directory. When calling the executable, pass the `-h` (e.g. `./generate_problem -h`) flag to see the command line arguments. 

## Release Setup/Installation
Forking the `https://github.com/peteramorese/AMP-Tools-public` is preferred, especially if you would like to commit your changes to GitHub. If you would not like to fork the repository, you can directly clone the repository.

**NOTE:** Please clone via `ssh` protocol if you are not doing so already. The submodule url uses the ssh-protocol, so even if you are able to clone the repository using HTTPS, you will not be able to update the submodule. If you have not setup and/or used a ssh key for github:

Creating an ssh key: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

Adding the ssh key to your github account: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account



Currently supported OS:
 - Ubuntu 20.04 (including WSL Ubuntu 20.04 on Windows)
 - Ubuntu 22.04 (including WSL Ubuntu 22.04 on Windows)
 - macOS x86
 - macOS arm64 

If your OS is not supported, and the project does not compile, please reach out to me (info at the bottom of the page).

---
### Ubuntu (or WSL)
On Ubuntu (or WSL), first install Eigen 3.3.7 (C++ matrix library), and OpenSSL, and the python3 GUI backend

```
sudo apt update
sudo apt install cmake build-essential libeigen3-dev libssl-dev python3-tk
```

Now install the `python3.x-dev` library for your system. To determine which version of python you are using:
```
python3 --version
```
For example, if you see `Python 3.10.6`, install `python3.10-dev`
```
sudo apt install python3.x-dev
```

To make sure you have a suitable version of Eigen
```
pkg-config --modversion eigen3
```
and verify the output is at least `3.3.7` (this should be the case if you are using Ubuntu 20.04 or newer)

Clone the forked repository anywhere you want:
```
git clone <forked AMP-Tools-public repo url>
cd AMP-Tools-public
```

Update the submodules (yaml-cpp)
```
git submodule update --init --recursive
```

Install the python dependencies
```
pip3 install -r requirements.txt 
```

Try building
```
bash build.sh
```

### macOS
Install `homebrew` if you do not already have it
```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Install CMake, Eigen 3.3.7, and OpenSSL. 
```
brew install cmake
brew install eigen
brew install openssl
```
The python headers should already be installed on your system.

Clone the forked repository anywhere you want:
```
git clone <forked AMP-Tools-public repo url>
cd AMP-Tools-public
```

Update the submodules (yaml-cpp)
```
git submodule update --init --recursive
```

Install the python dependencies
```
pip3 install -r requirements.txt 
```

Try building
```
bash build.sh
```

### Troubleshooting (macOS)
#### PythonLibs
The python headers should already be installed, however, if you receive a CMake error similar to
```
CMake Error... Cound NOT find PythonLibs
```
install the PythonLibs
```
brew install python3
```

---
## Submitting your homework
1) Each homework will have a `HW#::grade()` method that will test your code/algorithms and assess your score. Include this method somewhere in `main.cpp` (e.g. `HW2::grade(algo, "nonhuman.biologic@myspace.edu", argc, argv);`) and make sure to change your email to your colorado email. If you run main, you will now see logging output that shows the grading result when the tests are finished. No output file will be generated.
2) In order to generate the submission file `hw#_report_card.zip`, you will need to run `bash grade.sh --hw#` (e.g. for HW2: `bash grade.sh --hw2`). This will recompile your workspace and run your main function. If you don't see any errors, you should then find a report card file in `/file_dump/out/hw#_report_card.zip`.
3) Submit this `hw#_report_card.zip`. If you are in WSL, you may want to move this file outside of the WSL environment and into Windows to submit it (this may help: https://stackoverflow.com/questions/42586120/copy-files-from-windows-to-windows-subsystem-for-linux-wsl)

Do not edit/tamper with the `hw#_report_card.zip` file, doing so is a violation of the Honor Code. However, I've spent (way too many) hours making sure this process is literally Fort Knox-grade tamper-proof so even if you have the inclination to test your academic integrity, good luck!! >:D


---
## Issues, bugs, comments
Please feel free to raise issues on the GitHub page as they arise. If you experience an issue and are unsure if it is caused by a bug, don't hesitate to reach out to me at `peter.amorese@colorado.edu`.

I really hope you enjoy this toolbox!! - Peter

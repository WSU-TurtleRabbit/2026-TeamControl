#! /bin/bash
set -euo pipefail

### THIS SCRIPT REQUIRES (GIT) BASH TERMINAL ###

# Check if virtual environment exists
VENV_DIR=".venv"

# Delete existing venv (if any)
if [ -d "$VENV_DIR" ]; then
  rm -rf "$VENV_DIR"
fi

# Create venv (choose python command)
PYTHON_CMD="python3"
# if it is windows, use python instead.
case "$OSTYPE" in
  cygwin*|msys*|win32*)
    PYTHON_CMD="python"
    ;;
esac


if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # sudo apt update
    sudo apt-get install -y \
                 python3-venv python3-dev python3-tk
fi

# Create venv
$PYTHON_CMD -m venv "$VENV_DIR" || { echo "Cannot create virtual environment. Install Python 3.9+ (recommended 3.11)."; exit 1; }

# Activate venv
case "$OSTYPE" in
  linux-gnu*|darwin*)

    source "$VENV_DIR/bin/activate"
    ;;
  cygwin*|msys*|win32*)
    source "$VENV_DIR/Scripts/activate"
    ;;
  *)
    echo "Unsupported OS type: $OSTYPE"
    exit 1
    ;;
esac


    # if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Add Deadsnakes repo
        # sudo apt update
        # sudo apt install -y software-properties-common
        # # sudo add-apt-repository -y ppa:deadsnakes/ppa
        # sudo apt update
        # sudo apt-get install -y \
        #         python3-venv python3-dev python3-tk
        # #     python3.13 python3.13-venv python3.13-dev python3.13-tk
        # python3 -m venv "$VENV_DIR" || { echo "cannot initiate virtual environment."; exit 1; }
        # # python3.13 -m venv "$VENV_DIR" || { echo "cannot initiate virtual environment."; exit 1; }
    # fi

# # Check OS and use the correct activate script path
# if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
#     # linux and macOS
#     source "$VENV_DIR/bin/activate"
#     # pip install setuptools

    
# elif [[ "$OSTYPE" == "cygwin" ]] || [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]]; then
#     # For Git Bash on Windows
#     source "$VENV_DIR/Scripts/activate"
#     # For PowerShell on Windows: 
#     # ./$VENV_DIR/Scripts/activate.ps1

# else
#     echo "Unsupported OS. Could not activate the virtual environment."
#     exit 1
# fi
echo "--------------Upgrading pip/build tools...---------------------"
python -m pip install --upgrade pip setuptools wheel

echo "Installing project (editable) + testing extras..."
python -m pip install -e ".[testing]"
echo ""
echo "-------------------------------Done----------------------------------"
echo "Virtual environment summary"
echo "---------------------------------------------------------------------"

echo "Python:"
which python
python --version

echo ""
echo "Pip:"
python -m pip --version

echo ""
echo "Installed packages:"
python -m pip list

echo "---------------------------------------------------------------------"
echo "Activate Virtual Environment later with:"
echo "  source $VENV_DIR/bin/activate        # Linux/macOS"
echo "  source $VENV_DIR/Scripts/activate    # Git Bash on Windows"
echo "  ./$VENV_DIR/Scripts/activate.ps1     # for Windows PowerShell"

echo "---------------------------------------------------------------------"
echo "if on windows, powershell is not activating virtual environment do : "
echo "Set-ExecutionPolicy RemoteSigned -Scope CurrentUser"
echo "then restart powershell, and it'd work."


# echo -e "\n - - - Performing Git Pull - - - "
# git pull || { echo "Git pull failed"; exit 1; }

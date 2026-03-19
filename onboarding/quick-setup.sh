#!/bin/bash

# OEM IT Onboarding Quick Setup Script
# v2.1 - 03-19-2026
# 
# Authors:
# - Jack Greenberg - 09-24-2022
# - Henry Tejada Deras - 09-07-2025
# - Jacob Likins - 02-16-2026
# - Microsoft Copilot + GitHub Copilot
#
# Assumptions:
# - Ubuntu 24.04.3 LTS or later
# 
# CLI Arguments:
# <script path> [help] - Displays CLI Arguments Help
# <script path> [arm/avr/both] - Selects which firmware related packages to install associated with its toolchain; can be used in conjunction with --silent
# <script path> [--silent] - Runs all commands without user prompts (except for Conda TOS and sudo); installs both ARM and AVR toolchains and associated setups

# Declarations #################################################
set -u

# Colors
red=$(printf '\033[0;31m')
green=$(printf '\033[0;32m')
blue=$(printf '\033[34m')
white=$(printf '\033[97m')
bold=$(printf '\033[1m')
cl=$(printf '\033[0m')

# Configuration Variables
TMP_DIR=~/Downloads/oem-quick-setup-temp
VERSION='v2.1 - 03-19-2026'

# Temporary Files for State Management (used to keep state across shell restarts)
SILENT_MODE_FLAG="$TMP_DIR/silent_mode_flag.txt"
STATE_FILE="$TMP_DIR/script_state.txt"
TOOLCHAIN_SELECTION="$TMP_DIR/toolchain_selection.txt"

# Default Values
SILENT_MODE=false
STATE="Start"
TOOLCHAIN="both"

# Functions ####################################################

confirm_and_run() {
    cmd=$1
    if [[ "$SILENT_MODE" == true ]]; then
        printf "\n${green}Running: ${blue}${bold}$cmd${cl}\n"
        eval $cmd
    else
        read -p "${white}Run ${green}${cmd}${white}? [y/n] " -n 1 -r resp
        printf "\n"

        case $resp in
            [yY][eE][sS]|[yY])
                printf "\n${green}Running: ${blue}${bold}$cmd${cl}\n"
                eval $cmd
                ;;
            [nN][oO]|[nN])
                printf "${red}${bold}Skipping ${green}$cmd${cl}\n"
                ;;
            *)
                echo "Invalid input...\n"
                exit 1
                ;;
        esac
    fi
}

cli_help() {
    printf "
Usage: quick-setup.sh [ARGUMENTS] [OPTIONS]...

OEM IT Onboarding Quick Setup Script.
${VERSION}

Arguments:
help                Show this message and exit.
<toolchain>         Selects which firmware related packages to install associated with its toolchain.
                    Options: arm avr both

Options:
--silent            Runs all commands without user prompts (except for Conda Setup, TOS, and sudo)

    "
    exit 1
}

# Main Script ##################################################
main () {
    # Handle Command Line Arguments
    if [ $# -gt 2 ]; then
        echo "Error: Too many arguments."
        cli_help
    fi

    case $# in
        0)
            # No arguments provided, proceed with default settings
            ;;
        1)
            if [[ "$1" == "help" ]]; then
                cli_help
            elif [[ "$1" == "--silent" ]]; then
                SILENT_MODE=true
            elif [[ "$1" == "arm" ]]; then
                TOOLCHAIN="arm"
            elif [[ "$1" == "avr" ]]; then
                TOOLCHAIN="avr"
            elif [[ "$1" == "both" ]]; then
                TOOLCHAIN="both"
            else
                echo "Error: Invalid argument."
                cli_help
            fi
            ;;
        2)
            if [[ "$1" == "help" || "$2" == "help" ]]; then
                cli_help
            fi

            if [[ "$2" == "--silent" ]]; then
                SILENT_MODE=true
            fi

            if [[ "$1" == "arm" ]]; then
                TOOLCHAIN="arm"
            elif [[ "$1" == "avr" ]]; then
                TOOLCHAIN="avr"
            elif [[ "$1" == "both" ]]; then
                TOOLCHAIN="both"
            else
                echo "Error: Invalid argument combination. One of the arguments must specify the toolchain (arm/avr/both)."
                cli_help
            fi
            ;;
    esac

    printf "\n${bold}Welcome to the OEM quick setup!${cl}"
    printf "\n${bold}$VERSION${cl}\n"

    # Read Previous Script State (Handles Shell Restarts)
    if [ -d "$TMP_DIR" ]; then # Check if the temporary directory already exists
        printf "\n${bold}Note: Resuming from previous state, this overrides your command line arguments.${cl}\n"
        # Check if the state file exists
        if [[ -f "$STATE_FILE" ]]; then
            # Read the state
            STATE=$(cat "$STATE_FILE")
        else
            # Initialize the state
            STATE="Start"
        fi

        # Check if the silent mode flag file exists
        if [[ -f "$SILENT_MODE_FLAG" ]]; then
            # Read the silent mode flag
            SILENT_MODE=$(cat "$SILENT_MODE_FLAG")
        else
            SILENT_MODE=false
        fi

        # Check if the toolchain selection file exists
        if [[ -f "$TOOLCHAIN_SELECTION" ]]; then
            # Read the toolchain selection
            TOOLCHAIN=$(cat "$TOOLCHAIN_SELECTION")
        else
            TOOLCHAIN="both"
        fi
    else
        # Initialize the state and silent mode flag and Create Directory with Temp files: ~/Downloads/oem-quick-setup-temp
        mkdir -p $TMP_DIR
        cd $TMP_DIR
        echo "$STATE" > "$STATE_FILE"
        echo "$SILENT_MODE" > "$SILENT_MODE_FLAG"
        echo "$TOOLCHAIN" > "$TOOLCHAIN_SELECTION"
    fi

    # Save the current state, silent mode flag, and toolchain selection to their respective files
    echo "$STATE" > "$STATE_FILE"
    echo "$SILENT_MODE" > "$SILENT_MODE_FLAG"
    echo "$TOOLCHAIN" > "$TOOLCHAIN_SELECTION"

    # Get Script File Directory
    SCRIPT_PATH=$(readlink -f "$0")
    printf "\nScript Location: ${blue}${bold}$SCRIPT_PATH${cl}\n"

    # Post Initialization State
    printf "Temporary Files Directory: ${blue}${bold}$TMP_DIR${cl}\n"
    printf "Current State: ${blue}${bold}$STATE${cl}\n"
    printf "Toolchain Selection: ${blue}${bold}$TOOLCHAIN${cl}\n"
    if [[ "$SILENT_MODE" == true ]]; then
        printf "${bold}Note: Script is running in Silent Mode. All commands will be executed without user prompts (except for Conda Setup, TOS, and sudo)\n"
    fi

    ##################
    # Initialization #
    ##################
    if [[ "$SILENT_MODE" == false ]]; then
        # Prompt user if they are ready to proceed with the setup
        printf "\n"
        read -p "Are you sure you ready to proceed? (Y/n): " response
        if [[ "$response" == [yY] ]]; then
            echo "Proceeding..."
        else
            echo "Setup canceled."
            exit 0
        fi
    fi

    # Update Package List
    printf "\n"
    printf "${green}${bold}Updating Package List...${cl}\n"
    sudo apt-get update
  
    # Determine action based on state
    case "$STATE" in
        "Start")
            # Install cURL if not already installed
            printf "\n${green}${bold}Checking for cURL...${cl}\n"
                if command -v curl &> /dev/null; then
                    echo "cURL is installed. Skipping installation of cURL."
                else
                    sudo apt install curl -y
                fi

            # Install Miniconda
            printf "\n${green}${bold}Installing Miniconda... Please accept all default settings!${cl}\n"
            if command -v conda &> /dev/null; then # Check if Miniconda is already installed
                echo "Miniconda is already installed. Skipping installation of Miniconda."
            else
                confirm_and_run "curl -L https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh > $TMP_DIR/miniconda.sh && \
                chmod -v +x $TMP_DIR/miniconda.sh && \
                cd $TMP_DIR && \
                ./miniconda.sh"
            fi

            echo "Step 2" > "$STATE_FILE"  # Save the next state
            printf "\n${green}${bold}Please restart your shell to apply changes and then re-run the script.${cl}\n"
            exit 0
            ;;
        "Step 2")
            # Check if conda command is available
            if ! command -v conda &> /dev/null; then
                printf "\n${red}${bold}Error: Conda command not found. Please ensure Miniconda is installed and restart your shell before re-running the script.${cl}\n"
                exit 1
            fi

            # Accept Conda TOS
            printf "\n${green}${bold}Accepting Conda TOS...${cl}\n"
            printf "Anaconda Terms of Service: https://www.anaconda.com/legal/terms/terms-of-service\n"
            printf "Anaconda Privacy Policy: https://www.anaconda.com/legal/privacy-policy\n"
            printf "You must accept the Terms of Service to proceed. By inputting "y", you are accepting the terms of service.\n"
            printf "\n"
            read -p "Do you accept Anaconda's Terms of Service? (Y/n): " response
            if [[ "$response" == [yY] ]]; then
                echo "You accepted Anaconda's Terms of Service. Continuing setup."
                conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
                conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
            elif [[ "$response" == [nN] ]]; then
                echo "You did not accept the Terms of Service. Exiting setup."
                exit 0
            else
                echo "Invalid input. Exiting setup."
                exit 0
            fi

            # Create OEM Conda Environment
            printf "\n${green}${bold}Creating OEM Conda Environment...${cl}\n"
            if conda env list | grep -q "^oem\s"; then # Check if the "oem" environment already exists
                echo "Conda environment 'oem' already exists. Skipping creation of 'oem' environment."
            else
                confirm_and_run "conda create -n oem python=3.10 -y"
            fi

            # Setting OEM Conda Environment to Auto-Activate on Shell Start
            printf "\n${green}${bold}Setting OEM Conda Environment to Auto-Activate on Shell Start...${cl}\n"
            conda config --set auto_activate_base false
            echo "conda activate oem" >> ~/.bashrc
            source ~/.bashrc

            echo "Restarts Done" > "$STATE_FILE"  # Save the next state
            printf "\n${green}${bold}Please restart your shell to apply changes and then re-run the script.${cl}\n"
            exit 0
            ;;
        "Restarts Done")
            # Check if 'oem' environment is active
            if [[ "$(basename "$CONDA_DEFAULT_ENV")" != "oem" ]]; then
                printf "\n${red}${bold}Error: The 'oem' Conda environment is not active. Please restart your shell and ensure the 'oem' environment is activated before re-running the script.${cl}\n"
                exit 1
            fi
            # Install Python packages for OEM work
            printf "\n"
            printf "${green}${bold}Installing Python packages for OEM work...${cl}\n"
            if python -m pip --version &> /dev/null; then # Check if pip is installed in the 'oem' environment
                echo "pip is already installed in the 'oem' environment. Skipping installation of pip."
            else
                conda install pip -y
            fi
            confirm_and_run "pip3 install cantools click pyyaml PyQt5 numpy"

            # Install Non-Python packages for OEM work
            printf "\n${green}${bold}Installing Non-Python packages for OEM work...${cl}\n"
            confirm_and_run "sudo apt install can-utils build-essential libxcb-xinerama0 -y"
            sudo apt-get update # Update Package List

            #######
            # GIT #
            #######
            # https://git-scm.com/book/en/v2/Getting-Started-Installing-Git
            printf "\n"
            printf "${green}${bold}Installing Git (latest stable version)...${cl}\n"
            if command -v git &> /dev/null; then # Check if Git is already installed
                echo "Git is already installed. Skipping installation of Git."
            else
                confirm_and_run "sudo apt install git-all -y"
            fi
            eval "git --version" # Verify Installation

            #########
            # Bazel #
            #########
            # https://bazel.build/install/ubuntu
            printf "\n"
            printf "${green}${bold}Installing Bazel (latest stable version)...${cl}\n"
            # Check if Bazel is already installed
            if command -v bazel &> /dev/null; then
                echo "Bazel is already installed. Skipping installation of Bazel."
            else
                # Get Bazelisk
                curl -L https://github.com/bazelbuild/bazelisk/releases/latest/download/bazelisk-linux-amd64 > $TMP_DIR/bazelisk
                chmod +x $TMP_DIR/bazelisk
                sudo mv $TMP_DIR/bazelisk /usr/local/bin/bazel
            fi
            eval "bazel version" # Verify Installation

            #########
            # KICAD #
            #########
            # https://www.kicad.org/download/linux/
            printf "\n"
            printf "${green}${bold}Installing KiCad (v9.0)...${cl}\n"
            # Check if KiCad is already installed
            if command -v kicad &> /dev/null; then
                echo "KiCad is already installed. Skipping installation of KiCad."
            else
                confirm_and_run "sudo add-apt-repository --yes ppa:kicad/kicad-9.0-releases && \
                sudo apt update && \
                sudo apt install kicad -y"
            fi

            ###########
            # VS CODE #
            ###########
            # Installed using Ubuntu's snap package manager
            printf "\n"
            printf "${green}${bold}Installing VS Code (latest stable version)...${cl}\n"
            if command -v code &> /dev/null; then # Check if VS Code is already installed
                echo "VS Code is already installed. Skipping installation of VS Code."
            else
                confirm_and_run "sudo snap install --classic code"
            fi

            ################################################
            # FIRMWARE RELATED PACKAGES - ATmega 16M1/64M1 #
            ################################################
            if [[ "$TOOLCHAIN" == "avr" || "$TOOLCHAIN" == "both" ]]; then
                printf "\n"
                printf "\n${green}${bold}Installing firmware related packages for ATmega 16M1/64M1...${cl}\n"
                confirm_and_run "sudo apt install gcc-avr avrdude avr-libc binutils-avr gdb-avr -y"
            fi

            ###################################################################
            # FIRMWARE RELATED PACKAGES - STM32G441KB/STM32G441CB/STM32G474RE #
            ###################################################################
            if [[ "$TOOLCHAIN" == "arm" || "$TOOLCHAIN" == "both" ]]; then
                printf "\n"
                printf "\n${green}${bold}Installing firmware related packages for STM32G441KB/STM32G441CB/STM32G474RE...${cl}\n"
                confirm_and_run "sudo apt install gdb-multiarch openocd -y"
            fi

            #########
            # SLACK #
            #########
            # Installed using Ubuntu's snap package manager
            printf "\n"
            printf "${green}${bold}Installing Slack (latest stable version)...${cl}\n"
            if command -v slack &> /dev/null; then # Check if Slack is already installed
                echo "Slack is already installed. Skipping installation of Slack."
            else
                confirm_and_run "sudo snap install slack"
            fi

            ########################
            # MISC. UBUNTU CONFIG. #
            ########################
            printf "\n"
            printf "\n${green}${bold}Correcting Time Differences Between Windows and Ubuntu...${cl}\n"
            confirm_and_run "sudo timedatectl set-local-rtc 1"

            ############
            # CLEAN UP #
            ############
            # Remove Temporary Files Directory
            printf "\n"
            printf "${green}${bold}Cleaning Up Temporary Files...${cl}\n"
            rm -rf $TMP_DIR
            ;;
    esac

# End of Setup Script Message
cat << EOF
Congratulations! Your computer has now all the necessary packages and programs to work on most projects related to Olin Electric Motorsports! Next, you will need to set up your local Git and KiCad settings which is covered in the Software Setup section of the setup instructions.
EOF

}

main "$@"
#!/usr/bin/env bash
#
# Install script for Linux based Speech Recognition System
#
# Arpit Aggarwal
# 2018 TU Eindhoven

# Usage Document
usage()
{
    echo "-----------------------------------------------------------------------------"
    echo -e "\e[35m\e[1m                  LASeR: Linux Automatic Speech Recognition \e[0m"
    echo "-----------------------------------------------------------------------------"
    echo "Usage: sudo ./setup [options] <values>"
    echo -e "Options:\n \
        -h | --help\n \
        --install-kaldi\n \
        --update-kaldi\n \
        --clean\n \
        --complete"
    echo
    echo "-----------------------------------------------------------------------------"
}

if [ "$(/usr/bin/id -u)" -ne 0 ]; then
    echo
    echo "Error: Script must be executed as 'root'."
    echo
    echo "Check usage for more details."
    echo
    usage
    exit 1
fi

ASR_HOME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Export LASeR Environment Variables in .bashrc
# Check if the entry for .bash_exports exists in .bashrc
BASHEXPORTS=$( grep ".bash_exports" ~/.bashrc )

if [ -z "$BASHEXPORTS" ]
then
    echo -e "\e[35m\e[1m .bash_exports not found \e[0m"
    echo "
# Export definitions
if [ -f ~/.bash_exports ]; then
    . ~/.bash_exports
fi" >> ~/.bashrc
fi

if [ ! -f ~/.bash_exports ]
then
    echo "#! /usr/bin/env bash
" > ~/.bash_exports
fi

LASERENV=$( grep "$ASR_HOME/LASeR_env.bash" ~/.bash_exports )

if [ -z "$LASERENV" ]
then
    echo -e "\e[35m\e[1m LASeR_env.bash not found in .bash_exports \e[0m"
    echo "# Source Speech Recognition Environment
source $ASR_HOME/LASeR_env.bash
" >> ~/.bash_exports
fi

set -i
source ~/.bashrc
set +i

# Check if environment variables are sourced correctly
echo -e "\e[35m\e[1m Checking if .bashrc is sourced correctly \e[0m"
if [ -z $KALDI_ROOT ]
then
    echo -e "\e[34m\e[1m Variables not sourced \e[0m"
    exit -1
else
    echo -e "\e[34m\e[1m Variables sourced \e[0m"
    # exit 0
fi

# Setup script variables
KALDI=$KALDI_ROOT

# Logging function
log()
{
    echo "$(date "+%Y-%m-%d %H:%M:%S"): $1" >> $log_file
}

# Create Log directory
if [ ! -d "$ASR_LOG" ]
then
    mkdir $ASR_LOG
fi

# Change system timezone to Europe/Amsterdam [May not be required]
# sudo timedatectl set-timezone Europe/Amsterdam

# Install the required packages and dependencies
sudo apt-get update -qq > /dev/null 2>&1
sudo apt-get upgrade --assume-yes -qq > /dev/null 2>&1
sudo apt-get install --assume-yes build-essential git dphys-swapfile python python-scipy sox zip -qq > /dev/null 2>&1

# Install Postgresql only if required to
# postgresql postgresql-contrib python3-psycopg2

# Install festival for TTS
# festival

# Kaldi Build (Common to Installation and Update)
_kaldi_build()
{
    # Change exit to return to source check_dependencies and change back once done
    sed -i "s|exit|return|g" $KALDI/tools/extras/check_dependencies.sh
    source $KALDI/tools/extras/check_dependencies.sh
    sed -i "s|return|exit|g" $KALDI/tools/extras/check_dependencies.sh

    # Install dependencies
    echo -e "\e[36m\e[1m Installing dependencies \e[0m"
    sudo apt-get install libatlas3-base $debian_packages -qq > /dev/null
    sudo ln -s -f bash /bin/sh

    # Build toolkit
    echo -e "\e[36m\e[1m Building toolkit \e[0m"
    # Build the tools directory
    cd $KALDI/tools
    make -j 4 &> $ASR_LOG/make_tools.log
    make_tools_status=$( tail -n 1 $ASR_LOG/make_tools.log )

    if [ "$make_tools_status" != "All done OK." ]
    then
        echo -e "\e[34m\e[1m Make kaldi/tools failed \e[0m"
        exit 1
    fi

    extras/install_irstlm.sh &> $ASR_LOG/install_irstlm.log
    install_irstlm_status=$( grep "Installation of IRSTLM finished successfully" $ASR_LOG/install_irstlm.log )

    if [ -z "$install_irstlm_status" ]
    then
        echo -e "\e[34m\e[1m Install kaldi/tools/extras/install_irstlm.sh failed \e[0m"
        exit 1
    fi

    # Build the src directory
    cd $KALDI/src
    ./configure --shared &> $ASR_LOG/configure_src.log
    configure_src_status=$( grep "SUCCESS" $ASR_LOG/configure_src.log )

    if [ -z "$configure_src_status" ]
    then
        echo -e "\e[34m\e[1m Configure src failed \e[0m"
        exit 1
    fi

    make depend -j 4 > /dev/null
    make -j 4 &> $ASR_LOG/make_src.log
    make_src_status=$( grep "Done" $ASR_LOG/make_src.log )

    if [ -z "$make_src_status" ]
    then
        echo -e "\e[34m\e[1m Make src failed \e[0m"
        exit 1
    fi

    # Create a STATUS file to monitor installation
    echo -e "\e[36m\e[1m Kaldi installation complete \e[0m"	
    cd $KALDI
    echo "ALL OK" > STATUS
}

# Kaldi Installation
kaldi_install()
{
    echo -e "\e[36m\e[1m Checking for an existing Kaldi-ASR installation in $KALDI \e[0m"

    if [ ! -d "$KALDI" ]
    then
        # Clone repository into $KALDI
        echo -e "\e[36m\e[1m No existing installation found. Cloning from GitHub repository \e[0m"
        git clone --depth=1 https://github.com/kaldi-asr/kaldi.git $KALDI
        _kaldi_build
    else
        # Read STATUS file. If not "ALL OK", remove directory $KALDI and re-install Kaldi 
        kaldi_install_status="$(cat $KALDI/STATUS)"

        if [ "$kaldi_install_status" != "ALL OK" ]
        then
            sudo rm -rf $KALDI
            kaldi_install
        fi
    fi
}

# Kaldi Update
kaldi_update()
{
    if [ ! -d "$KALDI" ]
    then
        # Install Kaldi if directory $KALDI not present
    kaldi_install
    else
        # Read STATUS file. If "ALL OK" then update else remove directory $KALDI
        # and re-install Kaldi 
        kaldi_install_status="$(cat $KALDI/STATUS)"

        if [ "$kaldi_install_status" = "ALL OK" ]
        then
            # Pull changes from the repository
            echo -e "\e[36m\e[1m Updating repository from GitHub \e[0m"
            cd $KALDI
            git pull

            # Clean existing make
            echo -e "\e[36m\e[1m Cleaning existing make \e[0m"
            cd $KALDI/tools
            make distclean
            cd $KALDI/src
            make distclean

            # Build toolkit
            _kaldi_build

            echo -e "\e[36m\e[1m Kaldi-ASR update complete \e[0m"
        else
            sudo rm -rf $KALDI
            kaldi_install
        fi
    fi
}

# Clean the repository
setup_clean()
{
    sudo git clean -fdx
}

setup_complete()
{
    # TODO: Complete subroutine
    kaldi_update
}

# Read Postional Parameters
if [ -z "$1" ]
then
    usage
else
    while [ "$1" != "" ]
    do
        case $1 in
            --install-kaldi )
                kaldi_install ;;

            --update-kaldi )
                kaldi_update ;;

            --clean )
                setup_clean ;;

            --complete )
                setup_complete ;;

            -h | --help )
                usage
                exit 1 ;;

            * )
                usage
                exit 1 ;;
        esac
        shift
    done
fi


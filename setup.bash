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
	--update-kaldi"
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
	exit
fi

# Export LASeR Environment Variables
export ASR_HOME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export KALDI_ROOT=$ASR_HOME/kaldi
export ASR_LOG=$ASR_HOME/log
#log_file=$LOG_DIR/setup.log

# Setup script variables
KALDI=$KALDI_ROOT

# Logging function
log()
{
	echo "$(date "+%Y-%m-%d %H:%M:%S"): $1" >> $log_file
}

# Create Log directory
if [ ! -d "$LOG_DIR" ]; then
	mkdir $LOG_DIR
fi
#touch $log_file

# Change system timezone to Europe/Amsterdam
sudo timedatectl set-timezone Europe/Amsterdam

# Install the required packages and dependencies
sudo apt-get update -qq > /dev/null
sudo apt-get upgrade --assume-yes -qq > /dev/null
sudo apt-get install --assume-yes build-essential postgresql postgresql-contrib python3-scipy python3-psycopg2 ipython3 sox zip festival git aptitude dphys-swapfile

# Kaldi Installation
kaldi_install()
{
	echo "Checking for an existing Kaldi-ASR installation in $KALDI"
	if [ ! -d "$KALDI" ]
    then
		echo "No existing installation found. Initiating installation..."
		echo "Cloning repository from GitHub"
		git clone https://github.com/kaldi-asr/kaldi.git $KALDI
		
        # Change exit to return to source check_dependencies and change back once done
        sed -i "s|exit|return|g" $KALDI/tools/extras/check_dependencies.sh
		source $KALDI/tools/extras/check_dependencies.sh
		sed -i "s|return|exit|g" $KALDI/tools/extras/check_dependencies.sh
		
        log "Installing dependencies"
		sudo apt-get install libatlas3-base $debian_packages
		sudo ln -s -f bash /bin/sh
		
        cd $KALDI/tools
		log "Building toolkit"
		make -j 4
		extras/install_irstlm.sh
		cd ../src
		./configure --shared
		make depend -j 4
		make -j 4
		log "Kaldi installation complete"
		cd $KALDI
		echo "ALL OK" > STATUS
	else
		kaldi_install_status="$(cat $KALDI/STATUS)"
		if [ "$kaldi_install_status" != "ALL OK" ]
        then
			kaldi_install
		fi
	fi
}

# Kaldi Update
kaldi_update()
{
	if [ ! -d "$KALDI" ]
    then
		kaldi_install
	else
		kaldi_install_status="$(cat $KALDI/STATUS)"
		if [ "$kaldi_install_status" = "ALL OK" ]
        then
			cd $KALDI
			log "Updating repository from GitHub"
			git pull
			cd tools
			log "Cleaning existing make"
			make distclean
			log "Building toolkit"
			make -j 4
			extras/install_irstlm.sh
			cd ../src
			make distclean
			./configure --shared
			make depend -j 4
			make -j 4
			log "Kaldi-ASR update complete"
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
    echo -e "\e[35m\e[1mSetting up complete repository \e[0m"
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


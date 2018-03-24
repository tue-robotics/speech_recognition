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
	echo "                  LASeR: Linux Automatic Speech Recognition                  "
	echo "-----------------------------------------------------------------------------"
	echo "Usage: sudo ./setup [options] <values>"
	echo -e "Options:\n \
	-h | --help\n \
	--install-kaldi\n \
	--update-kaldi"
  echo
	echo "-----------------------------------------------------------------------------"
	echo "                     2018 Arpit Aggarwal, TU Eindhoven                       "
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

# Set Variables
ASR_HOME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$( cd "$( dirname "${ASR_HOME}" )" && pwd )"
KALDI=$ROOT/kaldi
LOG_DIR=$ASR_HOME/log
log_file=$LOG_DIR/setup.log

# Logging function
log()
{
	echo "$(date "+%Y-%m-%d %H:%M:%S"): $1" >> $log_file
}

# Create Log File
if [ ! -d "$LOG_DIR" ]; then
	mkdir $LOG_DIR
fi
touch $log_file
log "LASeR setup initiated"

# Change system timezone to Europe/Amsterdam
sudo timedatectl set-timezone Europe/Amsterdam
log "System timezone changed to Europe/Amsterdam"

# Install the required packages and dependencies
sudo apt-get update
log "Update complete"
sudo apt-get upgrade -y
log "Upgrade complete"
sudo apt-get install -y postgresql postgresql-contrib build-essential python3-scipy python3-psycopg2 ipython3 sox zip festival git aptitude dphys-swapfile
log "Installation of packages 'postgresql postgresql-contrib build-essential python3-scipy python3-psycopg2 ipython3 sox zip festival git aptitude dphys-swapfile' complete"

# Kaldi: Installation/Update
kaldi_install()
{
	log "Checking for an existing Kaldi-ASR installation in $KALDI"
	if [ ! -d "$KALDI" ]; then
		log "No existing installation found. Initiating installation..."
		log "Cloning repository from GitHub"
		git clone https://github.com/kaldi-asr/kaldi.git $KALDI
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
		if [ "$kaldi_install_status" = "ALL OK" ]; then
			log "Found valid Kaldi installation in $KALDI"
		else
			kaldi_install
		fi
	fi
}

kaldi_update()
{
	if [ ! -d "$KALDI" ]; then
		kaldi_install
	else
		kaldi_install_status="$(cat $KALDI/STATUS)"
		if [ "$kaldi_install_status" = "ALL OK" ]; then
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

# Read Postional Parameters
if [ -z "$1" ]; then
	usage
else
	while [ "$1" != "" ]; do
	    case $1 in
			--install-kaldi )				kaldi_install
											;;
			--update-kaldi )				kaldi_update
											;;
	        -h | --help )           		usage
	                                		exit
	                                		;;
	        * )                     		usage
	                                		exit 1
											;;
	    esac
	    shift
	done
fi

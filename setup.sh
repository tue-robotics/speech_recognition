#!/usr/bin/env bash
#
# Install script for Linux based Speech Recognition System
#
# Arpit Aggarwal
# 2018 TU Eindhoven
#
# Changes to be made:
# 2. Add crawling and dumping script to mandi_database_setup
# 3. Set daily crawling
# 4. Wanpipe install script

# Usage Document
usage()
{
	echo "-----------------------------------------------------------------------------"
	echo "                     Linux based Speech Recognition                          "
	echo "-----------------------------------------------------------------------------"
	echo "Usage: sudo ./setup [options] <values>"
	echo -e "Options:\n \
	-h | --help\n \
	--install-database <ip address> <username> <password>\n \
		PG Database server information\n \
		    Server IP Address\n \
		 	 	PG Username\n \
			 	PG User password\n \
	--install-kaldi\n \
	--install-asterisk\n \
	--install-pri-support\n \
	--install-hindi-numbertts"
	echo
	echo "-----------------------------------------------------------------------------"
	echo "                   2018 Arpit Aggarwal, TU Eindhoven                         "
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
jharkhand_mandi="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
root="$( cd "$( dirname "${jharkhand_mandi}" )" && pwd )"
call_flow_script=$jharkhand_mandi/arpit/jharkhand_farmer_system.php
pathSH=$jharkhand_mandi/arpit/path.sh
mandi_data=$jharkhand_mandi/arpit/data
crawl_data=$mandi_data/crawl_data
call_details=$mandi_data/call_details
asterisk_monitor_path=$call_details/monitor
responses=$call_details/responses
responses_jh=$responses/jharkhand
kaldi_dir=$root/kaldi
tts_dir=/usr/share/festival/voices/hindi
tts_dir2=/usr/share/festival/lib/voices/hindi

pg_hba_file=/etc/postgresql/9.5/main/pg_hba.conf
pg_config_file=/etc/postgresql/9.5/main/postgresql.conf
pg_pass_file=$HOME/.pgpass
log_file=$jharkhand_mandi/setup.log

pathSH_search="if [ -f path.sh ]; then . ./path.sh; fi"
pathSH_replace="if [ -f "$pathSH" ]; then . ./"$pathSH"; fi"
search_host_pg_hba="host    all             all             127.0.0.1/32            md5"
replace_host_pg_hba="host    all             all             0.0.0.0/0            md5"
search_address_pg_config="#listen_addresses = 'localhost'"
replace_address_pg_config="listen_addresses = '*'"
pg_host=
pg_user=
pg_pass=
pg_db=dit_speech_project
ext_file_search_call_flow=";CALL_FLOW_SCRIPT;"
ext_file_search_repo_home=";REPOSITORY_HOME;"
ext_file_search_mntr_path=";MONITOR_PATH;"

# Create Log File
touch $log_file
echo "$(date "+%Y-%m-%d %H:%M:%S"): Jharkhand Mandi ASR setup initiated" > $log_file

# Change system timezone to Asia/Kolkata
sudo timedatectl set-timezone Asia/Kolkata
echo "$(date "+%Y-%m-%d %H:%M:%S"): System timezone changed to Asia/Kolkata" > $log_file

# Install the required packages and dependencies
sudo apt-get update
echo "$(date "+%Y-%m-%d %H:%M:%S"): Update complete" >> $log_file
sudo apt-get upgrade -y
echo "$(date "+%Y-%m-%d %H:%M:%S"): Upgrade complete" >> $log_file
sudo apt-get install -y postgresql postgresql-contrib php build-essential python3-scipy python3-psycopg2 ipython3 sox php-pgsql zip festival git aptitude dphys-swapfile
echo "$(date "+%Y-%m-%d %H:%M:%S"): Install of packages 'postgresql postgresql-contrib php build-essential python3-scipy python3-psycopg2 ipython3 sox php-pgsql zip festival git aptitude dphys-swapfile' complete" >> $log_file

# Kaldi: Installation/Update
kaldi_install()
{
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Kaldi installation initiated" >> $log_file
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Checking for an existing installation in $kaldi_dir" >> $log_file
	if [ ! -d "$kaldi_dir" ]; then
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Kaldi toolkit not found" >> $log_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Cloning repository from GitHub" >> $log_file
		git clone https://github.com/kaldi-asr/kaldi.git
		sed -i "s|exit|return|g" $kaldi_dir/tools/extras/check_dependencies.sh
		source $kaldi_dir/tools/extras/check_dependencies.sh
		sed -i "s|return|exit|g" $kaldi_dir/tools/extras/check_dependencies.sh
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Installing dependencies" >> $log_file
		sudo apt-get install libatlas3-base $debian_packages
		sudo ln -s -f bash /bin/sh
		cd $kaldi_dir/tools
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Building toolkit" >> $log_file
		make -j 4
		extras/install_irstlm.sh
		cd ../src
		./configure --shared
		make depend -j 4
		make -j 4
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Kaldi installation successful" >> $log_file
	else
		echo "Kaldi is already installed."
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Found valid Kaldi installation $kaldi_dir" >> $log_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Kaldi installation procedure terminated" >> $log_file
	fi
}

kaldi_update()
{
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Kaldi update procedure initiated" >> $log_file
	if [ ! -d "$kaldi_dir" ]; then
		kaldi_install
	else
		cd $kaldi_dir
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Updating repository from GitHub" >> $log_file
		git pull
		cd tools
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Cleaning existing make" >> $log_file
		make distclean
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Building toolkit" >> $log_file
		make -j 4
		extras/install_irstlm.sh
		cd ../src
		make distclean
		./configure --shared
		make depend -j 4
		make -j 4
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Kaldi update successful" >> $log_file
	fi
}

# Setup Postgresql DB
mandi_database_setup()
{
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Initiating Mandi DB Setup" >> $log_file
	if [ -z $pg_host ] && [ -z $pg_user ] && [ -z $pg_pass ]; then
		echo "$(date "+%Y-%m-%d %H:%M:%S"): No input DB parameters found. Using defaults" >> $log_file
		pg_host=localhost
		pg_user=postgres
		pg_pass=arpit@48h1

		if [ ! -z "$( sudo grep "$search_host_pg_hba" "$pg_hba_file" )" ]; then
			sudo sed -i "s|${search_host_pg_hba}|${replace_host_pg_hba}|g" $pg_hba_file
			echo "$(date "+%Y-%m-%d %H:%M:%S"): pg_hba.conf set to accept remote connection" >> $log_file
		fi

		if [ ! -z "$( sudo grep "$search_address_pg_config" "$pg_config_file" )" ]; then
			sudo sed -i "s|${search_address_pg_config}|${replace_address_pg_config}|g" $pg_config_file
			echo "$(date "+%Y-%m-%d %H:%M:%S"): postgresql.conf set to accept remote connection" >> $log_file
		fi
		sudo -u postgres psql -U $pg_user -d postgres -c "alter user "$pg_user" with password '"$pg_pass"';"
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Postgresql user '$pg_user' password changed successfully to $pg_pass" >> $log_file
	fi

	pg_pass_file_grep=""$pg_host":5432:\*:"$pg_user":"
	pg_pass_file_replace=""$pg_host":5432:*:"$pg_user":"$pg_pass""

# Change pg_pass_file_replace in elif. grep string is wrong

	if [ ! -f "$pg_pass_file" ]; then
		echo "$pg_pass_file_replace" > $pg_pass_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): .pgpass file created and pgpass string added to it" >> $log_file

	elif [ ! -z "$( sudo grep "${pg_pass_file_grep}${pg_pass}" "$pg_pass_file" )" ]; then
		echo "$(date "+%Y-%m-%d %H:%M:%S"): .pgpass file already has the required configuration" >> $log_file

	elif [ ! -z "$( sudo grep "$pg_pass_file_grep" "$pg_pass_file" )" ]; then
		pg_pass_file_search="$( sudo grep "$pg_pass_file_grep" "$pg_pass_file" )"
		sudo sed -i "s|${pg_pass_file_search}|${pg_pass_file_replace}|g" $pg_pass_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): password changed in .pgpass file" >> $log_file

	else
		echo "$pg_pass_file_replace" >> $pg_pass_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): pgpass string added to .pgpass file" >> $log_file
	fi
	sudo chmod 0600 $pg_pass_file
	sudo chown $SUDO_USER:$SUDO_USER $pg_pass_file
	sudo cp $pg_pass_file /root/
	echo "$(date "+%Y-%m-%d %H:%M:%S"): .pgpass file copied to /root/ directory" >> $log_file

	echo "$(date "+%Y-%m-%d %H:%M:%S"): Checking if DB "$pg_db" exists" >> $log_file
	if [ ! -z "$( psql -h "$pg_host" -U "$pg_user" -lt | cut -d \| -f 1 | grep -w "$pg_db" )"]; then
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Found DB "$pg_db"" >> $log_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Dropping DB "$pg_db"" >> $log_file
		psql -h $pg_host -U $pg_user -d postgres -c "drop database "$pg_db";"
	else
		echo "$(date "+%Y-%m-%d %H:%M:%S"): No existing DB "$pg_db" found" >> $log_file
	fi

	psql -h $pg_host -U $pg_user -d postgres -c "create database "$pg_db";"
	echo "$(date "+%Y-%m-%d %H:%M:%S"): DB "$pg_db" created" >> $log_file
	cd $jharkhand_mandi/server_config/db_setup
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Importing 'Agri_ASR' schema dump into "$pg_db"" >> $log_file
	psql -h $pg_host -U $pg_user -d $pg_db < dit_speech_project.sql
	cd tables
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Importing tables into DB "$pg_db"" >> $log_file
	for i in *.csv; do
		psql -h $pg_host -U $pg_user -d $pg_db <<EOF
		\copy "Agri_ASR"."${i/.csv}" from $i with csv header
EOF
	done

	echo "$(date "+%Y-%m-%d %H:%M:%S"): Importing tables into "$pg_db" complete" >> $log_file

	# Add crawling and dumping scripts. Set for daily crawl.
	# Use cron to schedule jobs
}

# Install LibPRI, DAHDI, Asterisk and Wanpipe
asterisk_install()
{
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Asterisk installation initiated" >> $log_file
	echo "$(date "+%Y-%m-%d %H:%M:%S"): Checking for an existing installation" >> $log_file
	if [ -z "$(service --status-all | grep -F asterisk)" ]; then
		echo "$(date "+%Y-%m-%d %H:%M:%S"): No existing installation found" >> $log_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Installing from source" >> $log_file
		cd $jharkhand_mandi
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Untarring source to default install location" >> $log_file
		tar -xzvf server_config/asterisk-certified-13.13-current.tar.gz
		sudo mv asterisk-certified-13.13-cert3 /usr/local/src
		sudo cp server_config/asterisk_config/install_ubuntu /usr/local/src/asterisk-certified-13.13-cert3/contrib/scripts
		cd /usr/local/src/asterisk-certified-13.13-cert3/contrib/scripts
		source install_ubuntu
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Installing dependencies" >> $log_file
		handle_debian
		install_unpackaged
		cd ../../
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Building from source" >> $log_file
		sudo ./configure --with-pjproject-bundled
		sudo make menuselect.makeopts
		sudo menuselect/menuselect --enable-category MENUSELECT_CHANNELS menuselect.makeopts
		sudo make
		sudo make install
		sudo make samples
		sudo make config
		sudo make install-logrotate
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Asterisk installation successful" >> $log_file
	else
		echo "Asterisk is already installed"
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Found valid Asterisk installation $kaldi_dir" >> $log_file
		echo "$(date "+%Y-%m-%d %H:%M:%S"): Asterisk installation procedure terminated" >> $log_file
	fi

}

pri_support()
{
	cd $jharkhand_mandi
	tar -xzvf server_config/dahdi-linux-complete-current.tar.gz
	tar -xzvf server_config/libpri-current.tar.gz
	tar -xzvf server_config/wanpipe-7.0.20.tgz
	sudo mv dahdi-linux-complete-2.11.1+2.11.1 libpri-1.6.0 wanpipe-7.0.20 /usr/local/src
	cd /usr/local/src/dahdi-linux-complete-2.11.1+2.11.1
	sudo make
	sudo make install
	sudo make config
	cd ../libpri-1.6.0
	sudo make
	sudo make install
	asterisk_install
	# Add script to install Wanpipe
}


# Install IIT-Madras Number TTS
iitm_numbertts_setup()
{
	if [ ! -d "$tts_dir/iitm_hin_numbertts_clunits" ]; then
		cd $jharkhand_mandi
		unzip server_config/iitm_hin_numbertts_clunits.zip
		if [ ! -d "$tts_dir" ]; then
			sudo mkdir $tts_dir /usr/share/festival/lib /usr/share/festival/lib/voices $tts_dir2
		fi
		sudo mv iitm_hin_numbertts_clunits $tts_dir/
		sudo cp -r $tts_dir/iitm_hin_numbertts_clunits $tts_dir2/
	else
		echo "IITM Number TTS already installed"
	fi
}

# Prepare the required directories
mandi_setup()
{
	sudo mkdir $crawl_data $crawl_data/agmark $crawl_data/imd $crawl_data/log $call_details $asterisk_monitor_path $call_details/log $call_details/log/jharkhand $responses $responses_jh $responses_jh/choice $responses_jh/choice/dmp $responses_jh/choice/log $responses_jh/commodity $responses_jh/commodity/dmp $responses_jh/commodity/log $responses_jh/district $responses_jh/district/dmp $responses_jh/district/log $responses_jh/market $responses_jh/market/dmp $responses_jh/market/log $responses_jh/yesno $responses_jh/yesno/dmp $responses_jh/yesno/log $responses/log

	kaldi_install
	cd $kaldi_dir/egs/wsj/s5
	cp -r conf steps utils $jharkhand_mandi/arpit
	echo "--sample-frequency=8000" >> $jharkhand_mandi/arpit/conf/mfcc.conf
	sed -i "s|${pathSH_search}|${pathSH_replace}|g" $jharkhand_mandi/arpit/steps/make_mfcc.sh
	sed -i "s|${pathSH_search}|${pathSH_replace}|g" $jharkhand_mandi/arpit/steps/compute_cmvn_stats.sh

	asterisk_install
	cd /etc/asterisk
	sudo cat $jharkhand_mandi/server_config/asterisk_config/extensions.conf | sed -e "s|${ext_file_search_repo_home}|${jharkhand_mandi}|g; s|${ext_file_search_call_flow}|${call_flow_script}|g; s|${ext_file_search_mntr_path}|${asterisk_monitor_path}|g" >> extensions.conf
	sudo cat $jharkhand_mandi/server_config/asterisk_config/sip.conf >> sip.conf
	sudo cp $jharkhand_mandi/server_config/asterisk_config/chan_dahdi.conf chan_dahdi.conf

	mandi_database_setup
	iitm_numbertts_setup
}

# Read Postional Parameters
if [ -z "$1" ]; then
	mandi_setup
else
	while [ "$1" != "" ]; do
	    case $1 in
	        --install-database )			shift
	                                		pg_host=$1
											shift
											pg_user=$1
											shift
											pg_pass=$1
											mandi_database_setup
	                                		;;
			--install-asterisk )			asterisk_install
											;;
			--install-pri-support )			pri_support
											;;
			--install-kaldi )				kaldi_install
											;;
			--install-hindi-numbertts )		iitm_numbertts_setup
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

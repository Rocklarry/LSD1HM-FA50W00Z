#!/bin/bash

###############################################################
# step:
# 1. get all apps in bin/, sbin/, usr/bin, usr/sbin, and get
#    libraries these apps depend on.
# 2. get libraries these libraries(from 1st step) depend on.
# 3. get soft-link libraries if exist.
# 4. remove libraries that have not been used.
###############################################################

SH_NAME=$0
OP_FLAG=$1
ROOT_DIR=${2%/}	# rm '/' at the end of of a directory path if it has.

help_info()
{
	# get the basename of this scripts
	local shell_name=`basename $SH_NAME`

	echo -e "v1.1\n"
	echo -e "This script will downsize the rootfs or check the downsizing result!\n"
	echo -e "\033[32mUsage:\n\t./$shell_name <op_flag> <root_dir>\033[0m\n"
	echo -e "\t<op_flag>: operation.\n\t\t d - downsize rootfs\n\t\t c - check the downsizing result"
	echo -e "\t<root_dir>: the relative directory of your rootfs.\n"
}

################################################################################
# function: get the library dependency of source file
# $1: the source file.
# $2: the library dependency of source file.
# $3: the dlopen library dependency of source file.
################################################################################
get_libs()
{
	#readelf -s $ROOT_DIR/$app_name | grep "\<UND dlopen\>" > /dev/null 2>&1
	strings -a $1 | grep dlopen > /dev/null

	if [ $? -ne 0 ]; then
		local file_based_libs=`readelf -d $1 | grep NEEDED | sed -r 's/.*\[(.*)\].*/\1/'`
	else
		local file_based_libs_readelf=`readelf -d $1 | grep NEEDED | sed -r 's/.*\[(.*)\].*/\1/'`
		local file_based_libs=`strings -a $1 | grep -e "\.so\." -e "\.so$" | sed -r 's/.*\/(.*)/\1/'`

		for lib_name in $file_based_libs
		do
			echo $file_based_libs_readelf | grep $lib_name > /dev/null
			if [ $? -ne 0 ]; then
				echo "$lib_name" >> $3
			fi
		done
	fi

	echo "$file_based_libs" >> $2

	# check the existence of source-file's based libs.
	if [ "x$OP_FLAG" == "xd" ]; then
		for lib_name in $file_based_libs
		do
			if [ ! -f "$ROOT_DIR/lib/$lib_name" ]; then
				if [ ! -f "$ROOT_DIR/usr/lib/$lib_name" ]; then
					if [ -f "$3" ]; then
						grep $lib_name $3 > /dev/null
						if [ $? -eq 0 ]; then
							if [ ! -f "$ROOT_DIR/warning_file.txt" -a ! -f "$ROOT_DIR/warning_lib.txt" ]; then
								echo -e "\033[33m==WARNING==\033[0m file: $1，missing dlopen lib: $lib_name"
								echo "$1" >> $ROOT_DIR/warning_file.txt
								echo "$lib_name" >> $ROOT_DIR/warning_lib.txt
							else
								grep $1 $ROOT_DIR/warning_file.txt > /dev/null && grep $lib_name $ROOT_DIR/warning_lib.txt > /dev/null
								if [ $? -ne 0 ]; then
									echo -e "\033[33m==WARNING==\033[0m file: $1，missing dlopen lib: $lib_name"
									echo "$1" >> $ROOT_DIR/warning_file.txt
									echo "$lib_name" >> $ROOT_DIR/warning_lib.txt
								fi
							fi
						else
							echo -e "\033[31m==ERROR==\033[0m file: $1，missing lib: $lib_name"
							rm -f $ROOT_DIR/*.txt
							exit
						fi
					else
						echo -e "\033[31m==ERROR==\033[0m file: $1，missing lib: $lib_name"
						rm -f $ROOT_DIR/*.txt
						exit
					fi
				fi
			fi
		done
	fi
}

################################################################################
# function: get the library dependency of librires.
# $1: the file contains librires' name.
# $2: the library dependency of all librires in $1.
# $3: the dlopen library dependency of all librires in $1.
################################################################################
get_libs_extend()
{
	while read lib_name
	do
		if [ -f "$ROOT_DIR/lib/$lib_name" ]; then
			get_libs $ROOT_DIR/lib/$lib_name $2 $3
		else
			if [ -f "$ROOT_DIR/usr/lib/$lib_name" ]; then
				get_libs $ROOT_DIR/usr/lib/$lib_name $2 $3
			fi
		fi
	done < $1

	#keep every library only once
	sort -u	$2 -o $2

	if [ -f "$3" ]; then
		sort -u $3 -o $3
	fi
}

get_app_based_libs()
{
	#get all executable files.
	if [ -z "$2" ]; then
		local regular_file=`ls -l $ROOT_DIR/$1 | grep "^-" | awk '{printf("%s\n",$9)}' | sed "s/^/$1\//g"`
	else
		local regular_file=`ls -l $ROOT_DIR/$1/$2 | grep "^-" | awk '{printf("%s\n",$9)}' | sed "s/^/$1\/$2\//g"`
	fi

	for app_name in $regular_file
	do
		file $ROOT_DIR/$app_name | grep ELF | grep "executable" | grep "dynamically linked"  > /dev/null

		if [ $? -eq 0 ]; then
			get_libs $ROOT_DIR/$app_name $ROOT_DIR/app-based-libs.txt $ROOT_DIR/dlopen-libs.txt
		fi
	done
}

# get used libcedarx function
get_used_libcedarx()
{
	if [ -f $ROOT_DIR/etc/cedarx.conf ]; then
		local used_libcedarx=`grep "^l.*.so$" $ROOT_DIR/etc/cedarx.conf | awk '{print $3}'`

		for libcedarx_name in $used_libcedarx
		do
			if [ ! -f "$ROOT_DIR/lib/$libcedarx_name" ]; then
				if [ ! -f "$ROOT_DIR/usr/lib/$libcedarx_name" ]; then
					echo -e "\033[31m==ERROR==\033[0m can't find $libcedarx_name (refer to $ROOT_DIR/etc/cedarx.conf) in rootfs."
					rm -f $ROOT_DIR/*.txt
					exit
				fi
			fi
		done

		echo "$used_libcedarx" >> $ROOT_DIR/app-based-libs.txt
	fi
}

get_lib_based_libs()
{
	# for glibc, add /lib/libnss_* and libresolv*. refer to https://sourceware.org/ml/libc-help/2009-05/msg00046.html
	ls -l $ROOT_DIR/lib | grep -e "^-" -e "^l" | awk '{printf("%s\n",$9)}' | grep -e "^ld" -e "^lib" | grep -e 'libnss_' -e 'libresolv' >> $ROOT_DIR/app-based-libs.txt

	#keep every library only once
	sort -u $ROOT_DIR/app-based-libs.txt -o $ROOT_DIR/app-based-libs.txt

	cp $ROOT_DIR/app-based-libs.txt $ROOT_DIR/all-based-libs-tmp.txt

	#get lib based lib
	while true
	do
		cp -f $ROOT_DIR/all-based-libs-tmp.txt $ROOT_DIR/all-based-libs-tmp-diff.txt
		if [ -f "$ROOT_DIR/dlopen-libs.txt" ]; then
			cp -f $ROOT_DIR/dlopen-libs.txt $ROOT_DIR/dlopen-libs-diff.txt
		fi
		cp -f $ROOT_DIR/all-based-libs-tmp.txt $ROOT_DIR/app-based-libs.txt

		get_libs_extend $ROOT_DIR/app-based-libs.txt $ROOT_DIR/all-based-libs-tmp.txt $ROOT_DIR/dlopen-libs.txt

		diff $ROOT_DIR/all-based-libs-tmp.txt $ROOT_DIR/all-based-libs-tmp-diff.txt > /dev/null
		if [ $? -eq 0 ]; then
			diff $ROOT_DIR/dlopen-libs.txt $ROOT_DIR/dlopen-libs-diff.txt > /dev/null
			if [ $? -eq 0 ]; then
				rm -f $ROOT_DIR/all-based-libs-tmp-diff.txt $ROOT_DIR/dlopen-libs-diff.txt
				break
			fi
		fi
	done

	while read ol
	do
		if [ -f "$ROOT_DIR/lib/$ol" ]; then
			echo "lib/$ol" >> $ROOT_DIR/lib-based-libs.txt

			# for case: $ol -> $target_lib, get the $target_lib
			local target_lib=` ls -l $ROOT_DIR/lib/$ol | awk '{print $11}' `

			 # if $ol is a soft-link library, add the $target_lib library.
			if [ -n "$target_lib" ]; then
				echo "lib/$target_lib" >> $ROOT_DIR/lib-based-libs.txt
			fi

			# for case: $link_lib -> $ol, get the $link_lib (only the same dir)
			local link_lib=`ls -l $ROOT_DIR/lib | grep "^l.*$ol$" | awk '{print $9}'`

			# if $link_lib links to $ol, add the $link_lib library.
			if [ -n "$link_lib" ]; then
				echo "lib/$link_lib" >> $ROOT_DIR/lib-based-libs.txt
			fi
		else
			if [ -f "$ROOT_DIR/usr/lib/$ol" ]; then
				echo "usr/lib/$ol" >> $ROOT_DIR/lib-based-libs.txt

				# for case: $ol -> $target_lib, get the $target_lib
				local target_lib=` ls -l $ROOT_DIR/usr/lib/$ol | awk '{print $11}' `

				# if $ol is a soft-link library, add the $target_lib library.
				if [ -n "$target_lib" ]; then
					echo "usr/lib/$target_lib" >> $ROOT_DIR/lib-based-libs.txt
				fi

				# for case: $link_lib -> $ol, get the $link_lib (only the same dir)
				local link_lib=`ls -l $ROOT_DIR/usr/lib | grep "^l.*$ol$" | awk '{print $9}'`

				# if $link_lib links to $ol, add the $link_lib library.
				if [ -n "$link_lib" ]; then
					echo "usr/lib/$link_lib" >> $ROOT_DIR/lib-based-libs.txt
				fi
			fi
		fi
	done < $ROOT_DIR/all-based-libs-tmp.txt

#	sort -u $ROOT_DIR/lib-based-libs.txt
}

downsize_rootfs()
{
	ls -l $ROOT_DIR/lib | grep -e "^-" -e "^l" | awk '{printf("%s\n",$9)}' | grep -e "^ld" -e "^lib" | sed "s/^/lib\//g" >> $ROOT_DIR/libs.txt
	ls -l $ROOT_DIR/usr/lib | grep -e "^-" -e "^l" | awk '{printf("%s\n",$9)}' | grep -e "^ld" -e "^lib" | sed "s/^/usr\/lib\//g" >> $ROOT_DIR/libs.txt

	if [ "x$OP_FLAG" == "xd" ]; then
		rm -rf `cat $ROOT_DIR/lib-based-libs.txt $ROOT_DIR/libs.txt | sort | uniq -u | sed "s#^#$ROOT_DIR/#g"`
	fi

	if [ "x$OP_FLAG" == "xc" ]; then
		# for case $OP_FLAG is equal to 'c'
		local diff_lib=`cat $ROOT_DIR/lib-based-libs.txt $ROOT_DIR/libs.txt | sort | uniq -u`

		if [ -n "$diff_lib" ]; then
			echo -e "Check result:\033[31m==ERROR==\033[0m following libraries have not been used:\n$diff_lib"
		else
			echo -e "Check result:\033[32m==RIGHT==\033[0m"
		fi
	fi

	# rm unused minigui res
	if [ -f "$ROOT_DIR/usr/local/etc/MiniGUI.cfg" ]; then
		find $ROOT_DIR/usr/share/local/minigui -type f > $ROOT_DIR/minigui_res.txt

		grep -E '\.bmp|\.gif|\.cur|\.bin|\.ico|\.upf|\.vbf|\.name' $ROOT_DIR//usr/local/etc/MiniGUI.cfg | grep -v '^#' | sed -r 's/.*=(.*)/\1/' | sed -r 's/.*\/(.*)/\1/' > $ROOT_DIR/minigui_used_res.txt

		while read res_name
		do
			sed -i "/$res_name/d" $ROOT_DIR/minigui_res.txt
		done < $ROOT_DIR/minigui_used_res.txt

		if [ "x$OP_FLAG" == "xd" ]; then
			rm -rf `cat $ROOT_DIR/minigui_res.txt`
		fi

		if [ "x$OP_FLAG" == "xc" ]; then
			if [ ! -s "$ROOT_DIR/minigui_res.txt" ]; then
				echo -e "MiniGUI Check result:\033[32m==RIGHT==\033[0m"
			fi
		fi

	fi

	rm -f $ROOT_DIR/*.txt
}

###############################################################################
if [ $# -ne 2 ]; then
	help_info
	exit
fi

if [ "x$OP_FLAG" != "xd" ]; then
	if [ "x$OP_FLAG" != "xc" ]; then
		help_info
		echo -e "\t\033[31m==ERROR==\033[0m <op_flag>: '$OP_FLAG' should be 'd' or 'c'"
		exit
	fi
fi

if [ ! -d "$ROOT_DIR" ]; then
	help_info
	echo -e "\t\033[31m==ERROR==\033[0m <root_dir>: '$ROOT_DIR' does not exist!"
	exit
fi

if [ "x$OP_FLAG" == "xd" ]; then
	if [ -d "${ROOT_DIR}-tmp" ]; then
		echo "Directory ${ROOT_DIR}-tmp is exist!"
		cp -rf $ROOT_DIR/. ${ROOT_DIR}-tmp
	else
		echo "Directory ${ROOT_DIR}-tmp is not exist, back it up!"
		cp -narf $ROOT_DIR ${ROOT_DIR}-tmp
	fi
fi

get_app_based_libs bin
get_app_based_libs sbin
get_app_based_libs usr bin
get_app_based_libs usr sbin

get_used_libcedarx

get_lib_based_libs
downsize_rootfs

chmod 777 /home/sc/project/r11/tina30_0605/menred
cp /home/sc/project/r11/tina30_0605/libstdc++.so.6 /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/lib/
cp /home/sc/project/r11/tina30_0605/menred /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs
cp /home/sc/project/r11/tina30_0605/libQtCore.so.4 /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/lib/
cp /home/sc/project/r11/tina30_0605/libQtGui.so.4 /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/lib/
cp /home/sc/project/r11/tina30_0605/libQtNetwork.so.4 /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/lib/
cp /home/sc/project/r11/tina30_0605/led_test /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/
cp /home/sc/project/r11/tina30_0605/libts-1.3.so.0 /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/lib/
cp /home/sc/project/r11/tina30_0605/pointercal /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/
cp /home/sc/project/r11/tina30_0605/uart_gpio.ko /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/
cp /home/sc/project/r11/tina30_0605/lierda_beep.ko /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/
mkdir -p  /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/usr/local/Trolltech/QtEmbedded-4.8.7-arm/lib/fonts
chmod 777 /home/sc/project/r11/tina30_0605/kill.sh
chmod 777 /home/sc/project/r11/tina30_0605/crypt
chmod 777 /home/sc/project/r11/tina30_0605/c_success
chmod 777 /home/sc/project/r11/tina30_0605/system.ini
cp /home/sc/project/r11/tina30_0605/system.ini /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/etc/
cp /home/sc/project/r11/tina30_0605/c_success /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/
cp /home/sc/project/r11/tina30_0605/kill.sh /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/
cp /home/sc/project/r11/tina30_0605/crypt /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/etc/
cp /home/sc/project/r11/tina30_0605/AvantGardeBookBT.ttf /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/usr/local/Trolltech/QtEmbedded-4.8.7-arm/lib/fonts/
chmod 777 /home/sc/project/r11/tina30_0605/tianmao.png
cp /home/sc/project/r11/tina30_0605/tianmao.png /home/sc/project/r11/tina30_0605/out/banjo-R11_pref1/compile_dir/target/rootfs/etc


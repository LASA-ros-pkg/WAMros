#!/bin/bash
str=$0
str=`echo $str | sed s#.*/##`

if [ "$str" == "active.sh" ]
then
		match=1
		not=0
elif [ "$str" == "passive.sh" ]
then
		match=0
		not=1
else
		echo "Improper call of active.sh | passive.sh"
		exit
fi

str="["
for j in 1 2 3 4 5 6 7
do
	me=$not
	for arg in $*
	do
		if [ "$arg" == "$j" ] || [ "$arg" == "all" ]
		then
				me=$match
		fi
	done
	str=$str"$me"
	if [ "$j" == "7" ]
	then
			str=$str"]"
	else
			str=$str", "
	fi
done

echo "rosservice call /wam/active_passive '$str'"
eval "rosservice call /wam/active_passive '$str'"

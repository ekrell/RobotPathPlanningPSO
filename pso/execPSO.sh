MAP="OccupancyMap.txt"
cp $MAP /home/krell/Documents/robotPathPlanning/
cd /home/krell/Documents/robotPathPlanning
ARGS="-m $1 -a $2 -b $3 -c $4 -d $5 -e $6 -f $7 -n $8"  
/home/krell/Documents/robotPathPlanning/pathplanning $ARGS | grep 'PATH' | sed -e 's/PATH,//'  -e 's/,$//'

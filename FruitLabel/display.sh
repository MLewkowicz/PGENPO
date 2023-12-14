script_dir=/home/scrc/PGENPO/FruitLabel/
if [ -z "$1" ]
then 
echo "Missing Folder Path" 
exit 1
fi 

python3 ${script_dir}display_fruit.py $1
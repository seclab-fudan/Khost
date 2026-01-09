# dirs to be removed
dirs=(
    "./build"
    "./src/engine/runtime/build"
    "./src/engine/runtime/host"
    "./target"
)
for d in "${dirs[@]}"; do
  if [ -d "$d" ]; then
    rm -r "$d"
	((tt++))
  fi
done
tt=4
for d in "${dirs[@]}"; do
  if [ -d "$d" ]; then
	((tt--))
  fi
done
if [ $tt == 4 ]; then
	echo "Clean Successfully."
else
	echo "Fail to clean!"
fi

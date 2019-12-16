files=$(find src/ arduino/ -iname '*.hpp' -or -iname '*.cpp' -or -iname "*.h" -or -iname '*.cc' -or -iname '*.ino')

for f in $files; do
	echo $f
done

clang-format-8 $files -i

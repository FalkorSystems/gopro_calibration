#!/bin/sh
cd $1/Contents/Frameworks
FRAMEWORKS=*.framework
for f in ${FRAMEWORKS}
do
    codesign -s "Mac Developer Application" $f/Versions/Current
done
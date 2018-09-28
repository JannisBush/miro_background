#!/bin/sh

echo "blink"; cat "$1" | grep "blink" | wc -l
echo "inner brow raiser"; cat "$1" | grep "inner brow raiser" | wc -l
echo "outer brow raiser"; cat "$1" | grep "outer brow raiser" | wc -l
echo "brow lowerer"; cat "$1" | grep "brow lowerer" | wc -l
echo "upper lid raiser"; cat "$1" | grep "upper lid" | wc -l
echo "cheek raiser"; cat "$1" | grep "cheek raiser" | wc -l
echo "lid tightener"; cat "$1" | grep "lid tightener" | wc -l
echo "nose wrinkler"; cat "$1" | grep "nose wrinkler" | wc -l
echo "lip corner puller"; cat "$1" | grep "lip corner puller" | wc -l
echo "dimpler"; cat "$1" | grep "dimpler" | wc -l
echo "lip corner depressor"; cat "$1" | grep "lip corner depressor" | wc -l
echo "lip stretcher"; cat "$1" | grep "lip stretcher" | wc -l
echo "lip tightener"; cat "$1" | grep "lip tightener" | wc -l
echo "jaw drop"; cat "$1" | grep "jaw drop" | wc -l


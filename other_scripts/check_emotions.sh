#!/bin/sh

echo "happiness"; cat "$1" | grep "happiness detected" | wc -l
echo "sadness"; cat "$1" | grep "sadness detected" | wc -l
echo "surprise"; cat "$1" | grep "surprise detected" | wc -l
echo "fear"; cat "$1" | grep "fear detected" | wc -l
echo "anger"; cat "$1" | grep "anger detected" | wc -l
echo "disgust"; cat "$1" | grep "disgust detected" | wc -l
echo "contempt"; cat "$1" | grep "contempt detected" | wc -l

echo "blinkg game score: "; cat "$1" | grep "blinking game" 

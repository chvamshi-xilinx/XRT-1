#!/bin/bash

handler()
{
        echo "VAMSHI HERE"
        exit 0
}
trap handler SIGKILL SIGINT SIGTERM

while true;
do
        echo ""
done

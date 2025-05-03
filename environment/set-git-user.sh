#!/bin/bash

echo "Enter your full name:" 
read -r NAME
echo "Enter your email address:"
read -r EMAIL

git config --global user.name "$NAME" 

if [ $? -ne 0 ]; then
    echo "Failed to set git username"
    exit 1
fi

git config --global user.email "$EMAIL"

if [ $? -ne 0 ]; then
    echo "Failed to set git email"
    exit 1
fi

echo "git username and email set to $NAME and $EMAIL"
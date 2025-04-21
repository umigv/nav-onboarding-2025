#!/bin/bash

KEY_PATH="$HOME/.ssh/id_ed25519"

ssh-keygen -q -t ed25519 -N '' -f $KEY_PATH

if [ $? -eq 0 ]; then
    echo "SSH key generated and copied to clipboard"
else
    echo "Did not generate a new SSH key, existing key is copied to clipboard"
fi

xclip -selection clipboard < "$KEY_PATH.pub"
echo "Your SSH Public Key:"
cat $KEY_PATH.pub

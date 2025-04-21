#!/bin/bash

ssh-keygen -q -t ed25519 -N '' -f ~/.ssh/id_ed25519

if [ $? -eq 0 ]; then
    echo "Your SSH Public Key (copy the entire line with ctrl+shift+c):"
    cat ~/.ssh/id_ed25519.pub
else
    echo "Did not generate a new SSH key"
    echo "Your existing SSH Public Key (copy the entire line with ctrl+shift+c):"
    cat ~/.ssh/id_ed25519.pub
fi
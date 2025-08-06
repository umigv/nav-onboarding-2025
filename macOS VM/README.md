# macOS VM Setup

## First Time Setup

### Initializing VM

1. Download UTM @ https://mac.getutm.app/

2. Visit https://drive.google.com/drive/u/1/folders/1CuCsx1mUXYT3qFiAWCIg0p1zBhwae-vI

**Important**: Do **not** select everything and click download. The files/folders must be downloaded individually, otherwise it will be *very* slow.

3. Download each file/folder listed in the main folder. You should not open the folder, just download it (it will download as a zip)

4. Extract the zip file

5. Move `efi_vars.fd` and `VM_Data.qcow2` into the `Data` folder of the extracted `ARV VM macOS` folder

6. Rename the `ARV VM macOS` and add `.utm` to the end. Accept the warning that pops up.

7. Move the `.utm` file to somewhere safe

8. Double click the `.utm` file

### Setting up VM

1. Click the play button next to the newly imported VM.

2. Once the login screen shows, select "ARV Member"

3. The password of your user is `arvrules`

4. Once logged in, hit the Command key, type in `Terminal`, and hit Enter

5. Run `~/install_script.sh` and follow the prompts.
    
    Your `sudo` password is the same as your user password.
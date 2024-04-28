cd /home/sangam/Desktop
cp /home/sangam/Desktop/s2s/build/apn_phi-v1_default/apn_phi-v1_default.px4 /home/sangam/Desktop/saved_file
git init .
git add .
myVar = 1
msg = $myVar + "commit"
git commit -m msg
git branch -M main
git remote add origin https://github.com/sangam11a/ubuntu_data.git
git push -u origin main



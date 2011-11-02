mkdir _bulletBuild
cd _bulletBuild
wget http://bullet.googlecode.com/files/bullet-2.79-rev2440.tgz
tar -xvf bullet-2.79-rev2440.tgz
cd bullet-2.79-rev2440
sudo apt-get install cmake automake freeglut3
cmake . -G "Unix Makefiles" -SHARED_LIBS=ON 

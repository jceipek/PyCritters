mkdir _bulletBuild
cd _bulletBuild
wget http://bullet.googlecode.com/files/bullet-2.79-rev2440.tgz
tar -xvf bullet-2.79-rev2440.tgz
cd bullet-2.79-rev2440
sudo apt-get install build-essential libtool cmake automake freeglut3 freeglut3-dev
cmake . -G "Unix Makefiles" -SHARED_LIBS=ON 
./autogen.sh
./configure
make

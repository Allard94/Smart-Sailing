# Smart-Sailing
Smart Sailing project

## Setting up development environment

- Install JDK
 - sudo apt-get update
 - sudo apt-get install default-jdk

- Install Eclipse (https://eclipse.org/)
 - Choose "Eclipse IDE for C/C++ Developers"
 
- Install Git
 - sudo apt-get update
 - sudo apt-get install git

- Install libserialport (https://sigrok.org/wiki/Libserialport)
 - sudo apt-get install dh-autoreconf
 - clone libserialport (https://github.com/martinling/libserialport.git)
 - cd libserialport
 - ./autogen.sh
 - ./configure
 - make
 - sudo make install
 
- LibSBP installeren (https://swift-nav.github.io/libsbp/c/build/docs/html/install.html)
 - git clone https://github.com/swift-nav/libsbp.git
 - sudo apt-get install build-essential pkg-config cmake
 - cd libsbp/c/
 - mkdir build
 - cd build
 - cmake ../
 - make
 - sudo make install

# solver
This repo contains the code for the decentralized lagrangian solver 

## Installation
First install required depedencies:
```bash
sudo apt install build-essential
sudo apt install cmake
sudo apt install libblas-dev liblapack-dev
```

Then clone the repository
```bash
git clone git@github.com:ControlTrees/solver.git
```

The solver can be build using cmake
```bash
mkdir build && cd build
cmake ../solver
make
```

## Execute tests
All tests can be executed with the following command:
```bash
ctest
```

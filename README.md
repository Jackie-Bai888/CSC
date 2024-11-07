# CSC
This repository contains customized safety-critical scenarios generation framework, named CSC, which is capable of parsing the abstract textual descriptions of safety-critical scenarios supplied by testers and subsequently generating concrete instances of safety-critical scenarios. The CSC consists of scenario description translation and safety-critical scenario generation.

## Installation
1. Install [Apollo](https://github.com/ApolloAuto/apollo/tree/master) 
2. Install [Carla](https://carla.readthedocs.io/en/latest/)
3. To get all the dependencies, it is sufficient to run the following command.
```
pip install -r requirements.txt
```
## Getting Started
1. Get customized safety-critical scenarios using CSC method.
   ```
   python main.py
   ```
Note: You need to enter a description of the safety critical scenario in the input box. 

2. Get customized safety-critical scenarios using Random method.
   ```
   python main_random.py
   ```

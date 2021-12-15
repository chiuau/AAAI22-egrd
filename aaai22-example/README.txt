This folder contains the input and the output files for the examples in the paper.  We also include two more examples: example2-EP and example2-choice-vertex

Each folder contains the configuration file, the environment file, and the goal query statement file.  They are the inputs of our program. For more details about these files, please read the README file of our program.

We have generate the output log and the dot files by the following command:


  foctl -v -dot -s 5 false config.txt env.txt gqs.txt > output.log


The dot files can be opened by Graphviz.  We have converted the dot files to PDF files by the following commands:


  dot -Kneato -Tpdf -O env.dot

  dot -Tpdf -O gqs.dot


For more detail about the examples, please read the technical appendix.


Author: Tsz-Chiu Au
Email: chiu.au@gmail.com
Date: 2021-12-15



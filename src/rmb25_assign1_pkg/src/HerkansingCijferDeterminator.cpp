/*
Node description: what is the node doing , what are the node objects used 
Herkansing “Cijfer” Determinator
Subscriber + Service Client + Publisher + Timer + ActionClient + ActionServer

"
This node executes the a new final cijfer determination for a student/course combination
by receiving again randon exams results from the result generator. For this the random
generation has to be activated again by sending a message to the result generator node.
After receiving again enough results the exams are sent to the cijfer calculator to determine
a new final result. The new final result is added to the database (note: the old one should
not be overwritten).
"

*/ 


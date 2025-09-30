/*
Node description: what is the node doing , what are the node objects used 
Tentamen Result Generator Node
WallTimer + Publisher + Subscriber

"The tentamen result generator node collects from a Database (file) all student/course
combinations for which ”tentamen” results need to be generated. These are the
student/course combinations for which the final course result does not exist. After the
collection the node creates and broadcastes randomly an exam mark (between 10 and
100) for a random student/course combination (meaning e.g. every two seconds a result is
published for a random student/course combination. The node can receives a messages
that will ask to add or to remove a student/course combination to the random generation
process"
*/ 


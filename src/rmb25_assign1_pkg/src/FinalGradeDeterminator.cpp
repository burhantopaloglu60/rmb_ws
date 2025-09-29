/*
Node description: what is the node doing , what are the node objects used 
Final “Cijfer” Determinator
Subscriber + Service Client + Publisher

"The receives the tentamen results from the result generator. After it has collected enough
results (enough: = the collection number that was retrieved from the database) it sends a
request to the “cijfer” calculator node. After it got the final cijfer back it inserts the final
“cijfer” in the database and sends a message to the cijfer generator to stop the generation
for this student/course combination."
*/ 


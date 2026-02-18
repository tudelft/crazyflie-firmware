#ifndef NEURAL_NET_H
#define NEURAL_NET_H

/* Network dimensions */
#define INPUT_DIM 20
#define OUTPUT_DIM 4
#define NUM_LAYERS 4

/* Forward pass function */
void forward(const float* input, float* output);

#endif /* NEURAL_NET_H */

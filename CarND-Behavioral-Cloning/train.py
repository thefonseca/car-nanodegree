import model
import numpy as np

concept_model, discriminator, visual_generator, action_generator, generator_discriminator = model.init_model((40, 80, 1))

noise_image = np.random.rand(1, 1, 40, 80, 1)
noise_action = np.random.rand(1, 1, 1)

concept = concept_model.predict([noise_image, noise_action], verbose=0)

print(concept)
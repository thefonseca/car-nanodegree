import os
import csv
import model
import cv2
import numpy as np

# Fix error with Keras and TensorFlow
import tensorflow as tf
tf.python.control_flow_ops = tf

#from sklearn.model_selection import train_test_split

def load_samples():
    samples = []
    with open('../driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)
    
    return samples

def sensorimotor_generator(concept_model, visual_generator, action_generator,
                           frames_per_concept=5, batch_size=32, scale_factor=None,
                           grayscale=True, image_crop=(50, 30, 0, 0)):

    samples = load_samples()
    num_samples = len(samples)
    print('Number of samples:', num_samples)
    
    while 1: # Loop forever so the generator never terminates
        #shuffle(samples)
        
        for offset_batch in range(0, num_samples - (batch_size + frames_per_concept), 
                                  batch_size + frames_per_concept):
            
            real_images = []
            fake_images = []
            real_actions = []
            fake_actions = []
            concepts = []
            
            for offset in range(offset_batch, offset_batch + batch_size):
                concept_samples = samples[offset:offset+frames_per_concept]
                
                images = []
                angles = []

                for sample in concept_samples:
                    name = '../IMG/'+sample[0].split('/')[-1]
                    center_image = cv2.imread(name)
                    
                    if grayscale:
                        center_image = cv2.cvtColor(center_image, cv2.COLOR_BGR2GRAY)
                        
                    # trim image to only see section with road
                    center_image = center_image[image_crop[0]:center_image.shape[0]-image_crop[1],
                                                image_crop[2]:center_image.shape[1]-image_crop[3]]
                    
                    if scale_factor:
                        center_image = cv2.resize(center_image, None,
                                                  fx=scale_factor, fy=scale_factor, 
                                                  interpolation=cv2.INTER_CUBIC)
                    
                    center_angle = float(sample[3])
                    images.append(center_image)
                    angles.append(center_angle)

                images_tensor = np.array(images)
                
                # trim image to only see section with road
                #images_tensor = images_tensor[:,image_crop[0]:images_tensor.shape[1]-image_crop[1],
                #                              image_crop[2]:images_tensor.shape[2]-image_crop[3]]

                images_tensor = (images_tensor.astype(np.float32) - 127.5)/127.5
                
                if grayscale:
                    images_tensor = images_tensor.reshape((batch_size, frames_per_concept) + images_tensor.shape[1:] + (1,))
                else:
                    images_tensor = images_tensor.reshape((batch_size, frames_per_concept) + images_tensor.shape[1:])

                actions_tensor = np.array(angles)
                actions_tensor = actions_tensor.reshape((batch_size, frames_per_concept) + (1,))
                
                concept = concept_model.predict([images_tensor[:,:-1], actions_tensor[:,:-1]], verbose=0)
                #concepts.append(concept)
                
                image_prediction = visual_generator.predict(concept, verbose=0)
                action_prediction = action_generator.predict(concept, verbose=0)
                #print('image', image_prediction.shape)
                
                real_images_seq = images_tensor.reshape(images_tensor.shape[1:])
                fake_images_seq = np.concatenate((real_images_seq[:-1], image_prediction), axis=0)
                real_images.append(real_images_seq)
                fake_images.append(fake_images_seq)
                
                real_actions_seq = actions_tensor.reshape(actions_tensor.shape[1:])
                fake_actions_seq = np.concatenate((real_actions_seq[:-1], action_prediction), axis=0)
                real_actions.append(real_actions_seq)
                fake_actions.append(fake_actions_seq)
                
            #real = [np.array(real_images), np.array(real_actions)]
            #fake = [np.array(fake_images), np.array(real_actions)]
            real_images = np.array(real_images)
            real_actions = np.array(real_actions)
            fake_images = np.array(fake_images)
            fake_actions = np.array(fake_actions)
            #concepts = np.array(concepts)
            #concepts = concepts.reshape((batch_size, concepts.shape[-1]))
            
            yield (real_images, real_actions, 
                   fake_images, fake_actions)

def display_image(img, window, filename=None):
    img = img.reshape(img.shape[:-1])
    img = (img + 1) / 2.

    if filename:
        cv2.imwrite(filename, img*255)
        
    cv2.imshow(window, img)
    key = cv2.waitKey(1)
            
BATCH_SIZE = 1
FRAMES_PER_CONCEPT = 5
BATCH_COUNT = 100000

with tf.device('/gpu:0'):

    (concept_model, discriminator, visual_generator, 
     action_generator, generator_discriminator) = model.init_model((20, 80, 1), BATCH_SIZE)
    
    generator = sensorimotor_generator(concept_model, visual_generator,
                                       action_generator, scale_factor=.25,
                                       batch_size=BATCH_SIZE,
                                       frames_per_concept=FRAMES_PER_CONCEPT)

    for i in range(BATCH_COUNT):
        
        (real_images, real_actions, 
         fake_images, fake_actions) = next(generator)
        
        d_loss = discriminator.train_on_batch([real_images, real_actions], np.array([1,] * BATCH_SIZE))
        print("Batch", i, "d_loss_real_real=", d_loss)
        
        #d_loss = discriminator.train_on_batch([fake_images, fake_actions], np.array([0,] * BATCH_SIZE))
        #print("Batch", i, "d_loss_fake_fake=", d_loss)
        
        d_loss = discriminator.train_on_batch([fake_images, real_actions], np.array([0,] * BATCH_SIZE))
        print("Batch", i, "d_loss_fake_real=", d_loss)
        
        #d_loss = discriminator.train_on_batch([real_images, fake_actions], np.array([0,] * BATCH_SIZE))
        #print("Batch", i, "d_loss_real_fake=", d_loss)
        
        (real_images, real_actions, 
         fake_images, fake_actions) = next(generator)

        discriminator.trainable = False
        g_loss = generator_discriminator.train_on_batch([fake_images, fake_actions], 
                                                        np.array([1,] * BATCH_SIZE))
        discriminator.trainable = True
        
        print("Batch", i, "g_loss=", g_loss)
        
        if i > 0 and i % 200 == 0:
            display_image(real_images[-1][-1], 'real', filename='save/real-{}.png'.format(i))
            display_image(fake_images[-1][-1], 'fake', filename='save/fake-{}.png'.format(i))
        else:
            display_image(real_images[-1][-1], 'real')
            display_image(fake_images[-1][-1], 'fake')
        
        print("Actions:", real_actions[-1,-1], fake_actions[-1,-1])
            
        if i > 0 and i % 1000 == 0:
            visual_generator.save("save/model_visual_generator-{}".format(i), True)
            action_generator.save("save/model_action_generator-{}".format(i), True)
            discriminator.save("save/model_discriminator-{}".format(i), True)
            concept_model.save("save/model_concept-{}".format(i), True)

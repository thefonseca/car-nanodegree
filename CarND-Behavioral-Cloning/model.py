from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Activation, Flatten, Merge, Lambda, merge, Input
from keras.layers import Convolution2D, MaxPooling2D, UpSampling2D, BatchNormalization, Reshape
from keras.layers.recurrent import LSTM
from keras.layers.wrappers import TimeDistributed
from keras.optimizers import SGD

def visual_generator_model(image_shape):
    
    concept_input = Input(batch_shape=(1, 200), 
                          dtype='float', name='visual_generator_input')
    
    x = Dense(512)(concept_input)
    x = Activation('tanh')(x)
    
    input_shape = int(128*(image_shape[0]*image_shape[1])/16.)
    
    x = Dense(input_shape)(x)
    x = BatchNormalization()(x)
    x = Activation('tanh')(x)
    x = Reshape((int(image_shape[0]/4.), 
                                 int(image_shape[1]/4.), 128), 
                                input_shape=(input_shape,))(x)
    x = UpSampling2D(size=(2, 2))(x)
    x = Convolution2D(64, 5, 5, border_mode='same')(x)
    x = Activation('tanh')(x)
    x = UpSampling2D(size=(2, 2))(x)
    x = Convolution2D(image_shape[2], 5, 5, 
                                      border_mode='same')(x)
    x = Activation('tanh')(x)
    return Model(input=concept_input, output=x)

def action_generator_model():
    
    concept_input = Input(batch_shape=(1, 200), 
                          dtype='float', name='action_generator_input')
    
    x = Dense(100)(concept_input)
    x = Dense(1)(x)
    x = Activation('tanh')(x)
    return Model(input=concept_input, output=x)


def visual_encoder_model(image_shape):
    
    visual_input = Input(batch_shape=(1, None, image_shape[0], 
                                image_shape[1], image_shape[2]), 
                        dtype='float', name='visual_input')
    
    x = TimeDistributed(Convolution2D(
                        64, 5, 5,
                        border_mode='same'),
                        batch_input_shape=(1, None, image_shape[0], 
                                           image_shape[1], 
                                           image_shape[2]))(visual_input)
    x = TimeDistributed(Activation('tanh'))(x)
    x = TimeDistributed(MaxPooling2D(pool_size=(2, 2)))(x)
    x = TimeDistributed(Convolution2D(128, 5, 5))(x)
    x = TimeDistributed(Activation('tanh'))(x)
    x = TimeDistributed(MaxPooling2D(pool_size=(2, 2)))(x)
    x = TimeDistributed(Flatten())(x)
    #x = TimeDistributed(Dense(512))(x)
    #x = TimeDistributed(Activation('tanh'))(x)
    #x = TimeDistributed(Dense(200))(x)
    #x = TimeDistributed(Activation('tanh'))(x)
    x = LSTM(512, stateful=False, return_sequences=True, 
             name='ltsm_visual_encoder_1')(x)
    x = LSTM(200, stateful=False, return_sequences=True, 
             name='ltsm_visual_encoder_2')(x)
    model = Model(input=visual_input, output=x)
    return model

def action_encoder_model():
    
    action_input = Input(batch_shape=(1, None, 1), 
                        dtype='float', name='action_input')
    
    #x = TimeDistributed(Dense(100), batch_input_shape=(1, None, 3))(action_input)
    #x = TimeDistributed(Activation('tanh'))(x)
    x = LSTM(100, stateful=False, return_sequences=True,
             name='ltsm_action_encoder_1')(action_input)
    model = Model(input=action_input, output=x)
    return model

def concept_encoder_model(image_shape):
    
    visual = visual_encoder_model(image_shape)
    action = action_encoder_model()
    
    x = merge(visual.outputs + action.outputs, mode='concat')
    x = LSTM(200, stateful=False, return_sequences=False,
            name='lstm_concept')(x)
    concept = Model(input=visual.inputs + action.inputs, output=x)
    return concept

def discriminator_model(concept):
    x = Dense(100)(concept.outputs)
    x = Dense(1)(x)
    x = Activation('sigmoid')(x)
    #model.summary()
    return Model(input=concept.inputs, output=x)

def generator_discriminator_model(visual_generator, 
                                  action_generator,
                                  discriminator, image_shape):
    discriminator.trainable = False
    
    visual_reshape = Reshape((1, image_shape[0], 
                              image_shape[1], 
                              image_shape[2]),
                             input_shape=(image_shape[0], 
                                          image_shape[1], 
                                          image_shape[2]))(visual_generator.outputs)
    action_reshape = Reshape((1, 1), input_shape=(1,))(action_generator.outputs)
    
    x = discriminator([visual_reshape, action_reshape])
    return Model(input=visual_generator.inputs + 
                 action_generator.inputs, output=x)    

def init_model(image_shape):

    concept_model = concept_encoder_model(image_shape)
    discriminator = discriminator_model(concept_model)

    visual_generator = visual_generator_model(image_shape)
    action_generator = action_generator_model()

    concept_model_copy = concept_encoder_model(image_shape)
    discriminator_copy = discriminator_model(concept_model_copy)
    generator_discriminator = \
        generator_discriminator_model(visual_generator, 
                                      action_generator,
                                      discriminator_copy, 
                                      image_shape)

    visual_generator.compile(loss='binary_crossentropy', optimizer="adam")
    action_generator.compile(loss='binary_crossentropy', optimizer="adam")
    generator_discriminator.compile(loss='binary_crossentropy', 
                                       optimizer='adam')

    discriminator.trainable = True
    d_optim = SGD(lr=0.0005, momentum=0.9, nesterov=True)
    concept_model.compile(loss='binary_crossentropy', optimizer=d_optim)
    discriminator.compile(loss='binary_crossentropy', optimizer=d_optim)
    
    return concept_model, discriminator, visual_generator, action_generator, generator_discriminator, discriminator_copy

'''def saccade_size_from_activation(activation):
    size = (activation + 1) / 2.
    size = int(size * (MAX_WINDOW_SIZE - MIN_WINDOW_SIZE) + 
               MIN_WINDOW_SIZE)
    return size

def normalize_saccade_size(size):
    normalized = (size - MIN_WINDOW_SIZE) / (MAX_WINDOW_SIZE - MIN_WINDOW_SIZE)
    normalized = normalized * 2 - 1
    return normalized

def saccade_center_from_activation(activation):
    center = (activation + 1) / 2.
    center = (center * MAX_CENTER).astype(int)
    return center

def normalize_saccade_center(center):
    normalized = center / MAX_CENTER
    normalized = normalized * 2 - 1
    return normalized'''
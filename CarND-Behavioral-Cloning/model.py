from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Lambda
from keras.layers import Convolution2D, MaxPooling2D

def normalize(image):
    return image / 127.5 - 1.

def driving_model():

    model = Sequential()
    # normalize inputs
    model.add(Lambda(normalize, input_shape=(20,40,3)))
    model.add(Convolution2D(16, 2, 2, border_mode='same', activation='relu'))
    model.add(MaxPooling2D((2, 2)))
    model.add(Convolution2D(32, 3, 3, border_mode='same',activation='relu'))
    model.add(MaxPooling2D((2, 2)))
    model.add(Flatten())
    model.add(Dropout(0.2))
    model.add(Dense(1024, activation='relu'))
    model.add(Dense(100,activation='relu' ))
    model.add(Dense(50, activation='relu'))
    model.add(Dense(10, activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(1,name='output'))
    return model
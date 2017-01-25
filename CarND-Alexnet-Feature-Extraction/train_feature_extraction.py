import pickle
import time
import tensorflow as tf
from sklearn.model_selection import train_test_split
from alexnet import AlexNet
from sklearn.utils import shuffle
import numpy as np

# TODO: Load traffic signs data.
data = pickle.load(open("train.p", "rb"))
features = data['features']
labels = data['labels']
nb_classes = 43
labels = (np.arange(nb_classes) == labels[:,None]).astype(np.float32)

# TODO: Split data into training and validation sets.
X_train, X_val, y_train, y_val = train_test_split(features, 
                                                    labels, 
                                                    test_size=0.2, 
                                                    random_state=42)

def eval_on_data(X, y, sess):
    total_acc = 0
    total_loss = 0
    for offset in range(0, X.shape[0], batch_size):
        end = offset + batch_size
        X_batch = X[offset:end]
        y_batch = y[offset:end]

        loss, acc = sess.run([loss_op, accuracy_op], feed_dict={x: X_batch, y: y_batch})
        total_loss += (loss * X_batch.shape[0])
        total_acc += (acc * X_batch.shape[0])

    return total_loss/X.shape[0], total_acc/X.shape[0]

with tf.device("cpu:0"):
    
    # TODO: Define placeholders and resize operation.
    x = tf.placeholder(tf.float32, shape=(None, 32, 32, 3))
    resized = tf.image.resize_images(x, (227, 227))

    # TODO: pass placeholder as first argument to `AlexNet`.
    fc7 = AlexNet(resized, feature_extract=True)
    # NOTE: `tf.stop_gradient` prevents the gradient from flowing backwards
    # past this point, keeping the weights before and up to `fc7` frozen.
    # This also makes training faster, less work to do!
    fc7 = tf.stop_gradient(fc7)

    # TODO: Add the final layer for traffic sign classification.
    shape = (fc7.get_shape().as_list()[-1], nb_classes)  # use this shape for the weight matrix
    fc8_weights = tf.Variable(tf.truncated_normal(shape, stddev=0.1))
    fc8_bias = tf.Variable(tf.zeros(nb_classes))

    logits = tf.matmul(fc7, fc8_weights) + fc8_bias

    # TODO: Define loss, training, accuracy operations.
    # HINT: Look back at your traffic signs project solution, you may
    # be able to reuse some the code.

    y = tf.placeholder(tf.float32, [None, nb_classes])
    loss_op = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits, y))
    opt = tf.train.AdamOptimizer()
    train_op = opt.minimize(loss_op, var_list=[fc8_weights, fc8_bias])
    correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(y, 1))
    accuracy_op = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

    # TODO: Train and evaluate the feature extraction model.

    epochs = 10
    batch_size = 128

    with tf.Session() as sess:
        sess.run(tf.initialize_all_variables())

        for i in range(epochs):
            # training
            X_train, y_train = shuffle(X_train, y_train)
            t0 = time.time()
            for offset in range(0, X_train.shape[0], batch_size):
                end = offset + batch_size
                sess.run(train_op, feed_dict={x: X_train[offset:end], y: y_train[offset:end]})

            val_loss, val_acc = eval_on_data(X_val, y_val, sess)
            print("Epoch", i+1)
            print("Time: %.3f seconds" % (time.time() - t0))
            print("Validation Loss =", val_loss)
            print("Validation Accuracy =", val_acc)
            print("")

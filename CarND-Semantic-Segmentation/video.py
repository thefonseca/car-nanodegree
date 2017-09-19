from moviepy.editor import VideoFileClip, vfx
import tensorflow as tf
import os
import scipy.misc
import numpy as np
import main

with tf.Session() as sess:

    num_classes = 2
    data_dir = './data'
    vgg_path = os.path.join(data_dir, 'vgg')
    image_shape = (160, 576)

    saver = tf.train.import_meta_graph('./checkpoints/ckp-19.meta')
    saver.restore(sess, tf.train.latest_checkpoint('./checkpoints'))
    graph = tf.get_default_graph()
    logits = graph.get_tensor_by_name("logits:0")
    keep_prob = graph.get_tensor_by_name("keep_prob:0")
    image_input = graph.get_tensor_by_name("image_input:0")

    def process_image(image):
        # NOTE: The output you return should be a color image (3 channel) for processing video below
        # you should return the final output (image with lines are drawn on lanes)

        image = scipy.misc.imresize(image, image_shape)
        #image = np.asarray(image)
        print(image.shape)
        im_softmax = sess.run(
                [tf.nn.softmax(logits)],
                {keep_prob: 1.0, image_input: [image]})
        im_softmax = im_softmax[0][:, 1].reshape(image.shape[0], image.shape[1])
        segmentation = (im_softmax > 0.5).reshape(image.shape[0], image.shape[1], 1)
        mask = np.dot(segmentation, np.array([[0, 255, 0, 127]]))
        mask = scipy.misc.toimage(mask, mode="RGBA")
        street_im = scipy.misc.toimage(image)
        street_im.paste(mask, box=None, mask=mask)

        return np.array(street_im)


    clip = VideoFileClip('project_video.mp4')
    #clip = (VideoFileClip("project_video.mp4")
    #    .fx(vfx.resize, height=image_shape[0]))
    white_clip = clip.fl_image(process_image) #NOTE: this function expects color images!!
    white_clip.write_videofile('output.mp4', audio=False)
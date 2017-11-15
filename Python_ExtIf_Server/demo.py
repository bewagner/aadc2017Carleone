from tf_demo import tf_demo_init, classify_image, load_image_from_file

tf_demo_init()

result = classify_image(load_image_from_file('output2.jpg'))

print(result)

result = classify_image(load_image_from_file('images/a5_1.jpg'))

print(result)

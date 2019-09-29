import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import AllConstants

# read image
imBW = Image.open(AllConstants.floorImgAdress).convert(mode='L', dither=None)

# imBW = Image.open('Test2.jpg').convert(mode='L', dither=None)

ls = list(imBW.getdata())
imgS = imBW.size[::-1]
imgB = np.reshape(ls, imgS)
imgB_V1 = (np.round(imgB / 200))
imgB_V2 = imgB_V1 * -1 + 1

plt.imshow(imgB_V2, cmap='gray', interpolation='nearest')
y = set(imgB_V2.flat)

imgB_V2 = imgB_V2.transpose()
imgB_V2 = imgB_V2[:, ::-1]
plt.imshow(imgB_V2[:, ::-1].transpose(), cmap='hot', interpolation='nearest')
# plt.show()  ##Showing the imported image

print('Image', AllConstants.floorImgAdress, 'is imported as', imgS)
print('Image is converted to int array of', len(imgB_V2), '*', len(imgB_V2[0]),
      'with values consisted of', y)

# plt.imsave('FloorPro.png', imgB_V1.transpose(), cmap='gray')
plt.imsave('FloorPro.png', imgB_V1, cmap='gray')

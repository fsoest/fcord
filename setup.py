from setuptools import setup, find_packages

setup(
   name='fcord',
   version='0.1',
   packages=find_packages(),
   install_requires=[],
   description='A short description of your package',
   # long_description=open('README.md').read(),
   # long_description_content_type='text/markdown',
   # url='https://github.com/yourusername/your_package',
   author='Felix Soest',
   author_email='your.email@example.com',
   license='MIT',
   classifiers=[
       'Programming Language :: Python :: 3',
       'License :: OSI Approved :: MIT License',
       'Operating System :: OS Independent',
   ],
   python_requires='>=3.9',
)
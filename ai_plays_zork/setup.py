from setuptools import find_packages, setup

package_name = 'ai_plays_zork'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belca',
    maintainer_email='valeriobelcamino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zork_terminal_interaction = ai_plays_zork.zork_terminal_interaction:main', 
            'zork_inner_speech = ai_plays_zork.zork_inner_speech:main', 
            'zork_librarian = ai_plays_zork.zork_librarian:main',  
            'zork_decision_maker = ai_plays_zork.zork_decision_maker:main',  
        ]
    },
)

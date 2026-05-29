from setuptools import setup
setup(
    name="toppra-object-transport",
    packages=['transport',
              'transport.console',
    ],
    install_requires=[
        "PyYAML>=6.0",
    ],
    entry_points = {
        "console_scripts": [
            'transport.paper=transport.console.console_main:main',
        ]
    }
)

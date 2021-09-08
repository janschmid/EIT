# EIT-DAS  


**For devs that wish to push to the repository ; precommits are necessary, and an overview of how-to along with a styleguide is mentioned below**

## Precommits: 
Before a commit is accepted, a precommit is needed which tests the code for the following: Does it compile? Does it build? Does it comply with the given style guide ([Google c++ Style Guide](https://google.github.io/styleguide/cppguide.html) )


### Testing: 
Every branch, before it is merged to master, will have to go through pre-exisiting tests using googletest. 
If no test is available for your newly added feature, you are write some, [following this guide](https://google.github.io/googletest/primer.html)
(https://github.com/google/googletest)

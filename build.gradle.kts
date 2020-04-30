plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.0"
   vcsUrl = "https://github.com/ihmcrobotics/euclid-sandbox"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
    api("org.ejml:dense64:0.30")
    api("us.ihmc:ihmc-javafx-toolkit:0.12.7")
    api("us.ihmc:euclid-shape:0.14.0")
    api("us.ihmc:euclid-test:0.14.0")
}
plugins {
   id("us.ihmc.ihmc-build")
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
    api("us.ihmc:ihmc-javafx-toolkit:0.20.0")
    api("us.ihmc:euclid-shape:0.17.2")
    api("us.ihmc:euclid-test:0.17.2")
}
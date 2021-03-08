#!/bin/bash

# READ BEFORE USE 
# The script will download the individual pointcloud zip-files from the ecp webside using 'wget'.
# The zip-files are saved to a temporary location and will be extracted in the correct directory structure.
#
# Please adapt the following params:
# USER: your username from the ecp webside 
# PASS: your corresponding password
# EXTRACTION_ROOT: the directory the pointcloud data will be extracted to
# TMP_DOWNLOAD_LOCATION: the temporary location used to download the zip-files before extracting into $EXTRACTION_ROOT

USER="<your_username>"
PASS="<your_password>"

EXTRACTION_ROOT="./resources/data/"
TMP_DOWNLOAD_LOCATION="/tmp/"

# DO NOT CHANGE
BASE_URL="http://eurocity-dataset.tudelft.nl//eval/downloadFiles/downloadFile/ecpz?file=ecpdata%2Fecp2dot5%2Fecpz_pc%2Fper_city%2"

touch failed_to_download.log

while IFS=", " read -ra arr; do
  zip_file=${arr[0]}
  md5sum=${arr[1]}
  
  # Download zip
  wget --auth-no-challenge --user=$USER --password=$PASS --output-document=$TMP_DOWNLOAD_LOCATION$zip_file $BASE_URL$zip_file
  # Check md5sum of downloaded file
  md5_from_file=$(md5sum "$TMP_DOWNLOAD_LOCATION$zip_file" | cut -d " " -f1)
  if [[ $md5sum == $md5_from_file ]]
    then
      echo -e "\n\e[92mSUCCESS\e[39m"
    else
      echo -e "\n\e[91mFAILURE\e[39m\nMD5SUM missmatch for:\n$zip_file"
      echo $zip_file >> failed_to_download.log

      # Tidy up
      rm $TMP_DOWNLOAD_LOCATION$zip_file 
      continue
  fi

  # Extract and copy to correct location
  TIME=`echo $zip_file | cut -d "_" -f 1`
  SPLIT=`echo $zip_file | cut -d "_" -f 2`
  CITY=`echo $zip_file | cut -d "_" -f 3 | cut -d "_" -f 1`

  EXTR_DST=$EXTRACTION_ROOT$TIME"/pcs/"$SPLIT"/"$CITY
  if [[ ! -e $EXTR_DST ]]; then
    mkdir -p $EXTR_DST
  fi

  unzip $TMP_DOWNLOAD_LOCATION$zip_file -d $EXTR_DST
  rm $TMP_DOWNLOAD_LOCATION$zip_file 

echo "Done downloading. Check 'failed_to_download.log'"


done <<EOM
day_test_amsterdam.zip, 93e3372be3a0076099259fdb0a001fe1
day_test_barcelona.zip, 391a67031c853a5d51e8403d0ab306f7
day_test_basel.zip, 1c77d0be09df3c1bfb68deb9dc41f140
day_test_berlin.zip, 055b083e0fb1dba7fc1fa451cce8486c
day_test_bologna.zip, cc533b42c5fdc0c81f8dc54c2592247f
day_test_bratislava.zip, 0bf4d6a01be563cddd283b44b101d24f
day_test_brno.zip, 19d3bad83543d2164a29e237deda5505
day_test_budapest.zip, ab4d8bce6f4b0c0ad587fc1d86440fe7
day_test_dresden.zip, 4c089ab79bd7dc3f1b9f86e894d64f60
day_test_firenze.zip, 5ade71cb185d7c95586883c0da0b7c5c
day_test_hamburg.zip, 58634eb6069a42ec8afb4f28080a0128
day_test_koeln.zip, 5d09796a8b1312db06ce03150094b482
day_test_leipzig.zip, 946278175cbb5d691d8a5e4e9510d0ae
day_test_ljubljana.zip, 672e5e0f8b9b62ca05fe119562936370
day_test_lyon.zip, 0c95f251a4d8e3dcd7282a3a97441f48
day_test_marseille.zip, e6cb7bf029eeaaa5124f82b888f49438
day_test_milano.zip, 3b43d97ca7974e4d69b1c00f8598181f
day_test_montpellier.zip, 5b2fdc7e05d64a6d73201d8c755586df
day_test_nuernberg.zip, d618251228326c865258c0c2e1fc78ff
day_test_pisa.zip, b69cb5895b41b4f7afb2c37929ce401e
day_test_potsdam.zip, dc2f08a815e2d602867c661f07909250
day_test_prague.zip, 27e5eb91af63b8de0cfa58f7d0c7cbb5
day_test_roma.zip, 6cc780ac6886ceafbe8b9d5380ca6c78
day_test_stuttgart.zip, 00c4ca4bdefcb63cf37b3eaab84474f3
day_test_szczecin.zip, 16c2944859576cbd54b8e41acfe29927
day_test_torino.zip, 274d8832a603b34b65c0c5bf7ad646a8
day_test_toulouse.zip, 31b86aa2db4846917b5bd9906ffc098f
day_test_wuerzburg.zip, 57488542327fecaf14d072cc198e9634
day_test_zagreb.zip, df6600b9d15597227c415209adf95fca
day_test_zuerich.zip, 53257f386dbcd55ed538021a226e99b3
day_train_amsterdam.zip, a68fa4b60cc7c9e8ca119fd6beeeebfb
day_train_barcelona.zip, 3729e90fc309675e24d581987d4f7b2d
day_train_basel.zip, fa19cc99349d9205a0780c2c895e4d28
day_train_berlin.zip, 1815611281fb083ef3c462bba482b890
day_train_bologna.zip, 32f6757f1826ea37cf98883267f2a2c5
day_train_bratislava.zip, a80ec18787b6af0e38a6d8066be7a558
day_train_brno.zip, c7b1ef9d6d27772c630aa02c6e0657e9
day_train_budapest.zip, 87a455a005703a950e8829b265965f25
day_train_dresden.zip, 4d93b88b4e1e53bdce9a722152eb4d9c
day_train_firenze.zip, b1fc5c6668fd2eb58460ce6f3a0c09e5
day_train_hamburg.zip, 090bc100ff32ac48152ea0ee5dddc6ce
day_train_koeln.zip, 896eb32e6d471a73f6f15c7abcdb4b4d
day_train_leipzig.zip, 6cd131b13b2f76ea7ee9709eced1e326
day_train_ljubljana.zip, 3f37b5236142d77f0794cd95e3a07f10
day_train_lyon.zip, a7028b3bdbf4afde03529797735d8cd6
day_train_marseille.zip, 83c9bca2ac22c770ba9f2d478589c349
day_train_milano.zip, c554e938721026cf603a3632f7b87584
day_train_montpellier.zip, 65d0a29e605b739f1be978bd70b832c0
day_train_nuernberg.zip, 0411a754897c40bbdaa131b5edf0fb15
day_train_pisa.zip, 91f62efdc0492b8f443a5c7014fcfb9a
day_train_potsdam.zip, aa341546fde594b24611ce31fa6b6f58
day_train_prague.zip, 8f1b9a8c2700df8caceddc689d7e5eba
day_train_roma.zip, dbcfe7dfdb92adbdd8f0d06fa002c86b
day_train_stuttgart.zip, a216bfab1b0b2efd9da5671b7abba024
day_train_szczecin.zip, d6f5e019e9ca7027fdf2297577ce72a5
day_train_torino.zip, 888a74c43e148821dcf95018064bedec
day_train_toulouse.zip, a1fa5dd518eebd01df5b74d5db216a2f
day_train_wuerzburg.zip, 0d12020a69e0ea6102db7f0c56c044dd
day_train_zagreb.zip, 26d77c176bb4207833a0a01196273fd5
day_train_zuerich.zip, d33647a3cc26d35d96b33e8f2c1e3b34
day_val_amsterdam.zip, f047d6be83fdf60e8026ba0b6131ccca
day_val_barcelona.zip, 7045ed974c9f75cc42e4f3fa0088026c
day_val_basel.zip, 08fb6c8afc5f4c7bb724525702aff707
day_val_berlin.zip, 3ec6f6ee2af2634460f2741ca207da1b
day_val_bologna.zip, 273cf04f45e1fa2c1186e15a6cf389f1
day_val_bratislava.zip, bbc1bc00f8c2b6b6f985bf245e1d3060
day_val_brno.zip, 289baab67a89266086235adc33f8986d
day_val_budapest.zip, a6b92494ebe5b7dcb616602f6a7a25a9
day_val_dresden.zip, c064db05a8454facc478ea8f3da64ec8
day_val_firenze.zip, 826227c96683c5c8bc39e61d332d8903
day_val_hamburg.zip, 94a1fc2db6dda70234b6d206b5738fce
day_val_koeln.zip, b432d2d10ddeebaa0e9fe74175165454
day_val_leipzig.zip, e025e5badb0df9657290ec989a926063
day_val_ljubljana.zip, 4e028e93768d0c08893362edced898ff
day_val_lyon.zip, b3ed54b1ca7dd1a32b5b381c32723626
day_val_marseille.zip, 664b94ed1f2ce8944b04a85f3f2ece07
day_val_milano.zip, a1383097119bafb759a2d6ad980aabe9
day_val_montpellier.zip, 12bb46bc564e806ecd68692d9c9935f0
day_val_nuernberg.zip, 5c6f035040795a521bf57875606607ae
day_val_pisa.zip, 0c46bc46d66c67958be3c6bb3d69e65b
day_val_potsdam.zip, 57f9f874a00e1af2c341d724333533c6
day_val_prague.zip, 611e27bc09130361900ff390e51ab793
day_val_roma.zip, 093f8506abf714a97daa932b29ed56fb
day_val_stuttgart.zip, 56d4fc1a182dfc7670573475a6b4f672
day_val_szczecin.zip, 126acce6bd12494271069d65e01580bb
day_val_torino.zip, 52e29b8a3ad9f8ae9715e566f81c726f
day_val_toulouse.zip, e9f044b8b8927871277bb5d3fe57d95f
day_val_wuerzburg.zip, b7fb1eb333d6ef168f5b5584a16f8982
day_val_zagreb.zip, 70d5b2e4fb7ea84c6dceedc76de724d3
day_val_zuerich.zip, 9d41efc52b5ae2a15f9fe6d7f953abcf
night_test_budapest.zip, 1ee68ab50abee591ea407ee1a051d154
night_test_koeln.zip, 25f02836d0952d5dfb8ba152447ded27
night_test_leipzig.zip, 2e625d7a1e55011253feff3c29ab2e18
night_test_lyon.zip, a96c8e586ab08888bc8a1fd1f2b274ba
night_test_prague.zip, 6c74c587e982f16325590053c4e9d6c7
night_test_roma.zip, 2b94569577d83a5b540a0650103021cc
night_test_zagreb.zip, 49f1f2439e8546f607260b188c508138
night_train_budapest.zip, e96e16ff3db310f951b163eab878b716
night_train_koeln.zip, 2fbe7efd7cc2eeba73e1a7fbc2aa3f7a
night_train_leipzig.zip, c46b16447de70fcf44ac68fa705e0e10
night_train_lyon.zip, 29208fcd15deb949bd9c55d6f940d698
night_train_prague.zip, 60c6915b21d97a599198fd5d072bb39b
night_train_roma.zip, 63b46fb597376806ef0600ec4ef08b06
night_train_zagreb.zip, 569d10591f73129981ca0558955f0ac0
night_val_budapest.zip, 0c52d6be9a499f6402f46292fb580e38
night_val_koeln.zip, 4e4f1badb30a640c3b7e6f1235c65ad4
night_val_leipzig.zip, 1c2cdd7bf1c654932d0548e370c03313
night_val_lyon.zip, fc2b3d0714e17ce08c07bfde24b18ad9
night_val_prague.zip, bfbefb683ea7c58cc09521f9c10d6b14
night_val_roma.zip, 21cd23da5de236e836e2beed89b56d73
night_val_zagreb.zip, a6fe867f4266c3126308b5ea70934021
EOM



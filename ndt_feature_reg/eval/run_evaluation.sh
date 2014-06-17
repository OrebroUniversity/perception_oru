# MAIN FILE - contains everything required to perform the evaluation.
# Download the data
sh ./download_data.sh
# Unpack
sh ./extract_data.sh
# Create association.txt files
sh ./create_association_files.sh
# Generate all results
sh ./generate_all_results.sh
# Perform the evaluation towards the groundtruth using the TUM provided python scripts
sh ./perform_all_evaluations.sh

# Counting the utterance by a speaker

newSpeaker = False

while iterations:
    utterance = 1
    if not newSpeaker:
        row = next(csv_reader)

        sid = row[self.INDEX.SPEAKER_ID]
        uid = row[self.INDEX.UTTERANCE_ID]
        transcription = row[self.INDEX.TRANSCRIPTION]


        column1 = sid + 'U' + str(utterance)
        column2 = transcription

        out += column1 + '\t' + column2 + '\n'
        iterations -= 1
        utterance += 1

    else:
        sid = row[self.INDEX.SPEAKER_ID]
        transcription = row[self.INDEX.TRANSCRIPTION]

        column1 = sid + 'U' + str(utterance)
        column2 = transcription

        out += column1 + '\t' + column2 + '\n'
        iterations -= 1
        utterance += 1
        newSpeaker = False

    while (not newSpeaker) and iterations:
        row = next(csv_reader)
        if row[self.INDEX.SPEAKER_ID] != sid:
            newSpeaker = True
            break

        sid = row[self.INDEX.SPEAKER_ID]
        transcription = row[self.INDEX.TRANSCRIPTION]

        column1 = sid + 'U' + str(utterance)
        column2 = transcription

        out += column1 + '\t' + column2 + '\n'
        iterations -= 1
        utterance += 1

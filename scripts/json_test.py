import json 
import uuid

class NoIndent(object):
    def __init__(self, value):
        self.value = value


class NoIndentEncoder(json.JSONEncoder):
    def __init__(self, *args, **kwargs):
        super(NoIndentEncoder, self).__init__(*args, **kwargs)
        self.kwargs = dict(kwargs)
        del self.kwargs['indent']
        self._replacement_map = {}

    def default(self, o):
        if isinstance(o, NoIndent):
            key = uuid.uuid4().hex
            self._replacement_map[key] = json.dumps(o.value, **self.kwargs)
            return "@@%s@@" % (key,)
        else:
            return super(NoIndentEncoder, self).default(o)

    def encode(self, o):
        result = super(NoIndentEncoder, self).encode(o)
        for k, v in self._replacement_map.iteritems():
            result = result.replace('"@@%s@@"' % (k,), v)
        return result

if __name__ == "__main__":
	name = "test.log"
	with open(name, "r") as f:
		data = json.load(f)
		print(data)
	dict_template = {"col_id": NoIndent([0,4,5,6,6]), "timeStamp": [], "happy":[], "sad":[], "face_on_floor": []}
	d = [0,
		dict_template,
		dict_template]
	print(d)

	print(json.dumps(d, cls=NoIndentEncoder, indent=2))
	with open(name, "w") as f:
		f.write(json.dumps(d, cls=NoIndentEncoder, indent=2))

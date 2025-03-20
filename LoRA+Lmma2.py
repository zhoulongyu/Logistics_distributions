from transformers import LlamaForCausalLM, LlamaTokenizer, Trainer, TrainingArguments
from peft import get_peft_model, LoraConfig, TaskType


model_name = "meta-llama/Llama-2-7b-hf"  # 或者自定义Llama 3.3模型路径
tokenizer = LlamaTokenizer.from_pretrained(model_name)
model = LlamaForCausalLM.from_pretrained(model_name)


lora_config = LoraConfig(
    task_type=TaskType.CAUSAL_LM,
    r=8,
    lora_alpha=16,
    lora_dropout=0.1
)
model = get_peft_model(model, lora_config)


from datasets import load_dataset

dataset = load_dataset('csv', data_files='gazebo_uav_prompts.csv')

def tokenize_fn(examples):
    return tokenizer(examples['prompt'], truncation=True, padding='max_length', max_length=512)

tokenized_dataset = dataset.map(tokenize_fn, batched=True)

training_args = TrainingArguments(
    output_dir="./llama_uav_model",
    per_device_train_batch_size=4,
    num_train_epochs=3,
    logging_dir="./logs",
    save_strategy="epoch",
)


trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=tokenized_dataset['train'],
)


trainer.train()
